module core_model
import riscv_pkg::*;
(input  logic clk_i, rstn_i,          //instructor memory için CLK ve RST
 input  logic [XLEN-1:0] addr_i,
 output logic            update_o,
 output logic [XLEN-1:0] data_o,
 output logic [XLEN-1:0] pc_o,
 output logic [XLEN-1:0] instr_o,
 output logic [4:0]      reg_addr_o,
 output logic [XLEN-1:0] reg_data_o);


//XLEN ifadesi işlemcinin veriyolu genişliğini yani kaç bit olduğunu ifade ediyor. Burada XLEN 32 olduğu için 32 bitlik bir işlemciden bahsediyoruz.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//memory tanımlamaları
parameter int MEM_SIZE = 2048; // Memory tanımlamalarında bu parametreyi kullanacağız.
logic [31:0]     iMem [MEM_SIZE-1:0]; // instruction memory tanımlamasını yaptık
logic [31:0]     dMem [MEM_SIZE-1:0]; // data memory tanımlamasını yaptık
logic [XLEN-1:0] rf [31:0];       //32 bitlik 32 adet register file(rf) oluşturduk
initial $readmemh("./test/test.hex", iMem, 0, MEM_SIZE);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//program counter + instruction tanımlamaları
logic [XLEN-1:0] pc_d; // program counter'ın girişi. Yani sıradaki program counter değerini ifade eder.
logic [XLEN-1:0] pc_q; // program counter'ın çıkışı. Yani şu anki kullanılan program counter değerini ifade eder.
logic [XLEN-1:0] jump_pc_d; /* başka modüllerden program counter'ını başka bir değere ani olarak atlatmak için komut geldiğinde o modüller aynı zamanda bir program_counter(pc)
de gönderecek o zaman o gelen değişken bu jump_pc_d değişkenine aktarılıp bunun üzerinden kullanılacak.*/
logic            jump_pc_valid_d; // başka bir modülden komut gelip gelmediğini tutan kontrol değişkeni. Eğer komut geldiyse 1 olur gelmediyse 0 da kalır.
logic [XLEN-1:0] instr_d;  //giriş instruction(talimat/komut). Zaten instruction bir girdi olduğu için _d ile ifade edilir. Kısaca giriş kabloları _d, çıkış kabloları _q ile
//ifade edilir.
assign pc_o       = pc_q;
assign data_o     = dMem[addr_i];
assign instr_o    = instr_d;  //instr_d = iMem[pc_d[$clog2(MEM_SIZE) - 1 : 0]]; aşağıdaki programCounter_change_comb kısmında yandaki komut ile instr_d'yi okuduk zaten ve direkt instr_o'ya aktardık ki çıkışta görelim
assign reg_addr_o = rf_write_enable ? instr_d[11:7] : '0;
assign reg_data_o = rd_data;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
logic [XLEN-1:0] rs1_data;  //Register Source 1. İlk kaynak(source) register'ıdır.
logic [XLEN-1:0] rs2_data;  //Register Source 2. İkinci kaynak register'ıdır.
logic [XLEN-1:0] imm_data;  //İşlemciye register'dan değil de direkt verilen datayı ifade eder. immediate data
logic [4:0] shamt_data;     // bazı komutlarda kullandığımız shamt için kullandığımız datayı ifade eder.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
logic [XLEN-1:0] rd_data;   //Register Destination. İşlemin sonucunun yazılacağı register'dır.  Mesela, ADD rd,rs1,rs2 bu işlemde rs1 register'ı içindeki veri ile rs2 register'ı içindeki veriyi topla rd registerı
                            //içine yaz demek oluyor. ADD rd,rs1,10 ifadesindeki 10 değeri imm_data'yı ifade eder.
logic            rf_write_enable;   //register file write enable. Eğer bu 1 ise register file içindeki ilgili adrese(register destination) rd_data(register_destination data) yazılır.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
logic [XLEN-1:0] memory_write_data;     // Store aşamasında hafızaya yazacağımız veriyi ifade eder.
logic [XLEN-1:0] memory_write_addr; //Belleğin hangi adresine yazacaksak o adresi hesaplamak için gerekli değişken.
logic            memory_write_enable; // Hafızaya yazabilmemiz için hafızanın write enable'ının 1 olması gerekir.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////FETCH AŞAMASI//////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk_i or negedge rstn_i) begin  : programCounter_change_flipFlop  // program counter'ın değiştiği blok
    if(!rstn_i) begin
        pc_q <= 'h8000_0000; // eğer reset sinyali 0 ise program counter'ına 0 değeri atanır yani program counter'ı resetlenir. PC'yi 8000_0000 yapmamızın sebebi
                             // SPIKE SIMULATORU'nun RISC-V'da program counter'ı 0x8000_0000 adresinden başlatmasıdır.
        update_o <= 0;       // eğer program counter'ın şimdiki değerine(pc_q) sonraki program counter(pc_d) ataması yapılmadıysa update_o = 0 yapılarak işlemin gerçekleşmediği gösterilir.
    end
    else begin
        pc_q <= pc_d;       // eğer reset inyali 1 ise program counter'ına sıradaki program counter'ı atanır,
        update_o <= 1;     //eğer program counter'ın şimdiki değerine(pc_q) sonraki program counter(pc_d) ataması yapıldıysa update_o = 1 yapılarak bu işlemin gerçekleşmiş olduğu gösterilir.
    end

end

always_comb begin : programCounter_change_comb
    pc_d = pc_q;
    if(jump_pc_valid_d)        // eğer jump_pc_valid_d değeri 1 olduysa dışarıdan komut gelmiştir ve başka bir program counter(pc) gelmiştir o halde işlemci counter'ını normal
        pc_d = jump_pc_d;      // şekilde işletmek yerine dışarıdan gelen bu program counter'ı pc_d girişine atıyoruz ve bu sayede yeni program counter'ı çıkışa aktarılıyor 
    else                       // yukarıdaki pc_q <= pc_d sayesinde
        pc_d = pc_q + 4;       //burada ise yeni program counter eski program counter'ın üstüne + 4 eklenmiş hali olarak devam ediyor.
    
    instr_d = iMem[pc_q[$clog2(MEM_SIZE*4) - 1 : 2]]; /* Burada pc_q[$clog2(MEM_SIZE-1):2] ifadesi şunun için var, pc_q yukarıda [XLEN-1:0] şeklinde tanımlandı yani 32 bit, fakat bizim Memory Size'ımız 1024 yani
    adresleme yapmak için 32 bite değil yalnızca 10 bite ihtiyacımız var bu nedenle $clog2 fonksiyonunu kullanarak pc_q'yi 32 bitten 10 bite indirgiyoruz. Bu nasıl oluyor $clog2(parametre) fonksiyonu
    bir sayının 2 tabanındaki logaritmasını hesaplar, bu nedenle $clog2(MEM_SIZE-1) ifadesi MEM_SIZE değeri 1024 olduğu için 1024-1 = 1023 sayısının 2 tabanındaki logaritmasını hesaplar bu da 10'a eşittir.
    Bu durumda pc_q[$clog2(MEM_SIZE-1) - 1 : 0] ifadesi aslında pc_d[(10-1):0] => pc_q[9:0] şekline indirgenmiş oluyor yani pc_q(sıradaki program counter) değerinin SON 10 BİTİNE indirgenmiş olduk.*/

    /*NOT: instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 0]]; değil de instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; yapmamızın sebebi iMem'in logic [31:0]     iMem [MEM_SIZE-1:0]; şeklinde WORD ADRESLEME
    yapmasından kaynaklı yani iMem[0] = {byte3, byte2, byte1, byte0}, iMem[1] = {byte7,byte6,byte5,byte4} gibi. instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; risc-v'da bu şekilde alt iki biti çıkardığımız zaman 
    BYTE ADRESLEME MODUNDAN WORD ADRESLEME MODUNA GEÇİYORUZ. Eğer bunu yapmazsak mesela adres 00000004 olduğunda iMem[1]'e erişmesi gerekirken iMem[4]'e erişir bu da yanlıştır çünkü 00000004 ifadesi 4. byte'ı
    yani iMem[1]'in başlangıç adresini ifade eder.*/

    /* MEM_SIZE'ı 4 ile çarpmamızın sebebi, MEM_SIZE bize aslında KAÇ ADET WORD olduğunu gösteriyor yani 1024 adet WORD'umuz var fakat bir WORD İÇİNDE 4 BYTE OLDUĞU İÇİN  MEM_SIZE * 4 yapıyoruz ve belleğin
    toplam boyutunu BYTE CİNSİNDEN HESAPLIYORUZ Kİ PROGRAM COUNTER BYTE BYTE İLERLEDİĞİ İÇİN TOPLAM BYTE ALANINI HESAPLAMIŞ OLALIM.*/
end
///////////////////////////////////////FETCH AŞAMASI//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////DECODE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////
always_comb begin : decode_block   //instr_d içindeki instruction'un decode edilmesi, bu aşamada instr_d'nin belirli kısımlarındaki OPCODE'ların ne olduğuna göre case'ler açıp o case'ler içinde o OPCODE'nin görevi
                                   //neyse onları yapıcaz. Opcode dediğimiz kısım istr_d'nin son 7 biti yani instr_d[6:0] kısmı.
    
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    imm_data          = 32'b0;
    shamt_data        = 5'b0;
    rs1_data          = 32'b0;
    rs2_data          = 32'b0;
    memory_write_data = 32'b0;
    memory_write_addr = 32'b0;
    //---------------------------------------------------------------------------------------------------------------------------------------
    case(instr_d[6:0])
        OpcodeLui:                 //Lui = Load Upper Immediate
            imm_data = {instr_d[31:12], 12'b0};  //LUI komutunun tanımına göre imm_data'ya atama yaptık. Tanımda instruction'un 31:12 bitini al ve geri kalanına 0 ata diyor o nedenle biz de öyle yaptık.
        OpcodeAuipc:
            imm_data = {instr_d[31:12], 12'b0};   
        OpcodeJal: begin
            imm_data[31:21] = {11{instr_d[31]}};
            imm_data[20]    = instr_d[31];
            imm_data[10:1]  = instr_d[30:21];
            imm_data[11]    = instr_d[20];
            imm_data[19:12] = instr_d[19:12];
            imm_data[0]     = 1'b0; 
        end
        OpcodeJalr:
            if(instr_d[14:12] == F3_JALR) begin
                rs1_data = rf[instr_d[19:15]];
                imm_data[31:12] = {20{instr_d[31]}};
                imm_data[11:0]  = instr_d[31:20];
            end
        

        OpcodeBranch:                  //OpcodeBranch birçok operasyonda aynı olduğu için bir alt case içinde instr_d'nin farklı bir kısmına bakıp branch operasyonlarını ona göre ayırıyoruz.
            case(instr_d[14:12])       //FUNCT3 koduna bakıyoruz
                F3_BEQ, F3_BNE, F3_BLT, F3_BGE, F3_BLTU, F3_BGEU: begin // buradaki bütün case'ler için imm_data, rs1, rs2 ataması aynı burada fark YALNIZCA FUNCT3(instr_d[14:12]) kısmında olduğu için
                                                                //ayrım burada olucak BEQ,BNE, BLT, .... hangisiyse artık gerisi hepsi için aynı olduğu için böyle tek satırda yazdık.
                    rs1_data        = rf[instr_d[19:15]];
                    rs2_data        = rf[instr_d[24:20]];
                    imm_data[31:13] = {19{instr_d[31]}};   // riscv instruction pdf'indeki talimatlara göre IMM DATA'NIN HANGİ BİTİNİN HANGİ INSTRUCTION BITLERİNE KARŞILIK GELDİĞİNİ belirtiyoruz.
                    imm_data[12]    = instr_d[31];
                    imm_data[11]    = instr_d[7];
                    imm_data[10:5]  = instr_d[30:25];
                    imm_data[4:1]   = instr_d[11:8];
                    imm_data[0]     = 1'b0;
                end
                default: ;
            endcase
        OpcodeLoad:                  //aynı şekilde opcodebranch'daki olayın aynısını burada yapıyoruz.
            case(instr_d[14:12])     //FUNCT3 koduna bakıyoruz
                F3_LB, F3_LH, F3_LW, F3_LBU, F3_LHU  : begin
                    rs1_data        = rf[instr_d[19:15]];
                    imm_data[31:12] = {20{instr_d[31]}};
                    imm_data[11:0]  = instr_d[31:20];
                end
                default: ;
            endcase
        OpcodeStore:
            case(instr_d[14:12])     //FUNCT3 koduna bakıyoruz
                F3_SB, F3_SH, F3_SW   : begin
                    rs1_data        = rf[instr_d[19:15]];
                    rs2_data        = rf[instr_d[24:20]];
                    imm_data[31:12] = {20{instr_d[31]}};
                    imm_data[11:5]  = instr_d[31:25];
                    imm_data[4:0]   = instr_d[11:7];
                end
                default: ;
            endcase
        OpcodeOpImm:
            case(instr_d[14:12])     //FUNCT3 koduna bakıyoruz
            //ADD IMMEDIATE, sonunda I varsa Immediate, yoksa mesela F3_ADD ise normal ADD işlemi. Diğer kodlar için de böyle.
                F3_ADDI, F3_SLTI, F3_SLTIU, F3_XORI, F3_ORI, F3_ANDI : begin
                    rs1_data        = rf[instr_d[19:15]];
                    imm_data[31:12] = {20{instr_d[31]}};
                    imm_data[11:0]  = instr_d[31:20]; 
                end
                F3_SLLI :
                    if(instr_d[31:25] == F7_SLLI) begin
                        shamt_data = instr_d[24:20];
                        rs1_data   = rf[instr_d[19:15]];          
                    end
                F3_SRLI_SRAI :
                    if(instr_d[31:25] == F7_SRLI) begin
                        shamt_data = instr_d[24:20];
                        rs1_data   = rf[instr_d[19:15]];
                    end
                    else if(instr_d[31:25] == F7_SRAI) begin
                        shamt_data = instr_d[24:20];
                        rs1_data   = rf[instr_d[19:15]];
                    end
                default: ;
            endcase
        OpcodeOp:
            case(instr_d[14:12])
                F3_ADD_SUB :
                    if(instr_d[31:25] == F7_ADD) begin
                        rs1_data   = rf[instr_d[19:15]];
                        rs2_data   = rf[instr_d[24:20]];
                    end else if(instr_d[31:25] == F7_SUB) begin
                        rs1_data   = rf[instr_d[19:15]];
                        rs2_data   = rf[instr_d[24:20]];
                    end
                F3_SLL, F3_SLT, F3_SLTU, F3_XOR, F3_OR, F3_AND : begin
                    rs1_data   = rf[instr_d[19:15]];
                    rs2_data   = rf[instr_d[24:20]];
                end
                F3_SRL_SRA :
                    if(instr_d[31:25] == F7_SRL) begin
                        rs1_data   = rf[instr_d[19:15]];
                        rs2_data   = rf[instr_d[24:20]];                            
                    end else if(instr_d[31:25] == F7_SRA) begin
                        rs1_data   = rf[instr_d[19:15]];
                        rs2_data   = rf[instr_d[24:20]];
                    end
                default: ;
            endcase
        default: ;
    endcase
end
//////////////////////////////////////////DECODE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////EXECUTE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////
always_comb begin : execute_block  //
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    jump_pc_valid_d     = 0;
    jump_pc_d           = 0;
    rd_data             = 0;
    rf_write_enable     = 0;
    memory_write_enable = 0;
    memory_write_data   = 0;
    memory_write_addr   = 0;
    //----------------------------------------------------------------------------------------------------------------------------------------- 
    case(instr_d[6:0])
        OpcodeLui: begin
            rd_data = imm_data;          //register destination data, yani hedef register verisi imm_data olarak ayarlandı.
            rf_write_enable = 1'b1;      //rf_write_enable 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeAuipc: begin
            rd_data = pc_q + imm_data;   //register destination data, yani hedef register verisi imm_data + bir sonraki program counter olarak ayarlandı.
            rf_write_enable = 1'b1;
        end
        OpcodeJal: begin
            jump_pc_valid_d = 1'b1;
            jump_pc_d = imm_data + pc_q;
            rd_data = pc_q + 4;
            rf_write_enable = 1'b1;      //rf_write_enable 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeJalr: begin
            jump_pc_valid_d = 1'b1;
            jump_pc_d = imm_data + rs1_data;
            jump_pc_d[0] = 1'b0; // Burada en son bit 0 yapılıyor çünkü JALR komutunda en son bitin 0 olması gerekiyor.
            rd_data = pc_q + 4;
            rf_write_enable = 1'b1;      //rf_write_enable 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeBranch:
            case(instr_d[14:12])
             F3_BEQ: if(rs1_data == rs2_data) begin   // BRANCH IF EQUAL
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1;
             end
             F3_BNE: if(rs1_data != rs2_data) begin   // BRANCH IF NOT EQUAL
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1; 
             end 
             F3_BLT: if($signed(rs1_data) < $signed(rs2_data)) begin   // BRANCH IF LESS THAN, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1; 
             end 
             F3_BGE: if($signed(rs1_data) >= $signed(rs2_data)) begin   // BRANCH IF GREATER OR EQUAL, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1; 
             end 
             F3_BLTU: if(rs1_data < rs2_data) begin   // BRANCH IF LESS THAN(UNSIGNED)
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1; 
             end 
             F3_BGEU: if(rs1_data >= rs2_data) begin   // BRANCH IF GREATER OR EQUAL(UNSIGNED)
                jump_pc_d = pc_q + imm_data;
                jump_pc_valid_d = 1'b1; 
             end
             default: ; 
            endcase
        OpcodeLoad:
            case(instr_d[14:12])
             F3_LB: begin   //LOAD BYTE
                rd_data[31:8]   = {24{dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7]}}; //burada ise sign extension yapıyoruz çünkü 32 bitlik rd_data var ve biz alt satırda bu 32 bitlik data içine sadece
                                                                                 //dMem içerisindeki eriştiğimiz adres içindeki verinin son 8 bitini aktardık çünkü Load işlemleri sadece son 8 biti ister.

                rd_data[7:0]    = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7:0];     //rs1_data[$clog2[MEM_SIZE]-1:0] bu ifade ile rs1_data[9:0] şeklinde 10 bitlik bir değer gelicek ve bunun sayesinde dMem
                                                                                 //içerisindeki 0-1023 adresten birine erişicez sonrasında bu adres içindeki [7:0] 8 bitlik veriyi rd_data[7:0] a aktarıcaz çünkü
                                                                                 //LB(LOAD BYTE) yapıyoruz yani 1 byte(8 bit) lik bir yükleme yapıyoruz bu nedenle son 8 biti[7:0] çekiyoruz.
                rf_write_enable = 1'b1;
             end
             F3_LH: begin  //LOAD HALFBYTE
                rd_data[31:16]  = {16{dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15]}};//yine aynı şekilde sign extension yapıyoruz.
                rd_data[15:0]   = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                 //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
                rf_write_enable = 1'b1;
             end
             F3_LW: begin  //LOAD WORD
                rd_data         = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]];          //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LW(LOAD WORD) yapıyoruz yani 4 byte(32 bit)
                                                                                 //yükleme yapıyoruz, zaten 32 bit yüklediğimiz için herhangi bir EXTENSION YAPMAMIZA GEREK KALMIYOR.
                rf_write_enable = 1'b1;
             end
             F3_LBU: begin //LOAD BYTE UNSIGNED
                rd_data[31:8] = 24'b0;                                           //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli lakin burada LBU(UNSIGNED) olduğu için extension'u
                                                                                 //sign extension yapmıyoruz de ZERO(0) EXTENSION olarak yapıyoruz.
                rd_data[7:0]  = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7:0];
                rf_write_enable = 1'b1;
             end
             F3_LHU: begin //LOAD HALFWORD UNSIGNED
                rd_data[31:16]  = 16'b0;                                         //Yine SIGN EXTENSION YERINE ZERO EXTENSION YAPTIK, çünkü komut LHU(UNSIGNED) olarak ifade edilmiş gerisi LH ile aynı.
                rd_data[15:0]   = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                 //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
                rf_write_enable = 1'b1;
             end
             default: ;
            endcase
        OpcodeStore:
            case(instr_d[14:12])
             F3_SB: begin
                memory_write_enable  = 1'b1;
                memory_write_data    = rs2_data;            //yine risc-v instruction setindeki talimatlara göre memory_write_data = rs2_data atamasını yaptık.
                memory_write_addr    = rs1_data + imm_data; //rs1_data => rs1 register'ının adresi(base adres), imm_data => instruction'dan gelen offset(kayma) değeri, bu ikisi toplanınca memory_write_adress
                                                            //bulunmuş oluyor, bu adrese ise memory_write_data yani rs2_data değeri yazılıyor.
             end
             F3_SH: begin
                memory_write_enable  = 1'b1;
                memory_write_data    = rs2_data;  //yine risc-v instruction setindeki talimatlara göre memory_write_data = rs2_data atamasını yaptık.
                memory_write_addr    = rs1_data + imm_data;
             end
             F3_SW: begin
                memory_write_enable  = 1'b1;
                memory_write_data    = rs2_data;  //yine risc-v instruction setindeki talimatlara göre memory_write_data = rs2_data atamasını yaptık.
                memory_write_addr    = rs1_data + imm_data;
             end
             default: ;
            endcase
        OpcodeOpImm:
            case(instr_d[14:12])
             F3_ADDI: begin
                rf_write_enable = 1'b1;
                rd_data = $signed(imm_data) + $signed(rs1_data);
             end
             F3_SLTI: begin  //SET LESS THAN IMMEDIATE
                rf_write_enable = 1'b1;
                if($signed(rs1_data) < $signed(imm_data))
                    rd_data = 32'b1;
             end
             F3_SLTIU: begin //SET LESS THAN IMMEDIATE(UNSIGNED)
                rf_write_enable = 1'b1;
                if(rs1_data < imm_data)
                    rd_data = 32'b1;
             end
             F3_XORI: begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data ^ imm_data;
             end
             F3_ORI: begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data | imm_data;
             end
             F3_ANDI: begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data & imm_data;
             end
             F3_SLLI:     //SHIFT LEFT LOGICAL IMMEDIATE
                if(instr_d[31:25] == F7_SLLI) begin
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data << shamt_data;
                end
            F3_SRLI_SRAI:     
                if(instr_d[31:25] == F7_SRLI) begin          //SHIFT RIGHT LOGICAL IMMEDIATE
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data >> shamt_data;
                end else if(instr_d[31:25] == F7_SRAI) begin //SHIFT RIGHT ARITMATIC IMMEDIATE
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data >>> shamt_data;
                end
            default: ;
            endcase
        OpcodeOp:
            case(instr_d[14:12])
             F3_ADD_SUB:
                if(instr_d[31:25] == F7_ADD) begin
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data + rs2_data;
                end else if(instr_d[31:25] == F7_SUB) begin
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data - rs2_data;
                end
            F3_SLL:     begin   //SHIFT LEFT LOGICAL
                rf_write_enable = 1'b1;
                rd_data = rs1_data << rs2_data;
            end
            F3_SLT:     begin   //SET LESS THAN
                rf_write_enable = 1'b1;
                if($signed(rs1_data) < $signed(rs2_data))
                    rd_data = 32'b1;
            end
            F3_SLTU:    begin  //SET LESS THAN UNSIGNED
                rf_write_enable = 1'b1;
                if(rs1_data < rs2_data)
                    rd_data = 32'b1;
            end
            F3_XOR:     begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data ^ rs2_data;
            end
            F3_SRL_SRA: 
                if(instr_d[31:25] == F7_SRL) begin //SHIFT RIGHT LOGICAL
                    rf_write_enable = 1'b1;
                    rd_data = rs1_data >> rs2_data;
                end else if(instr_d[31:25] == F7_SRA) begin
                    rf_write_enable = 1'b1;
                    rd_data = $signed(rs1_data) >>> rs2_data; //Aritmetik şekilde sağa kaydırmak için hem >>> hem de $signed ifadesi kullanmamız lazım aynı anda
                end
            F3_OR: begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data | rs2_data;                  
            end
            F3_AND: begin
                rf_write_enable = 1'b1;
                rd_data = rs1_data & rs2_data;
            end
            default: ;
        endcase
        default: ;
    endcase
end
//////////////////////////////////////////EXECUTE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////MEMORY//////////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk_i or negedge rstn_i) begin   //Store işlemi için gerekli kod bloğu
    if(!rstn_i) begin
    end
    else if(memory_write_enable) begin
        //sanki instr_d[14:12] değeri aşağıdaki işlemlerden birine denk gelirse store işlemi olsa da olmasa da aşağıdaki işlemlerden biri gerçekleşiyor gibi gözüküyor fakat bu durumu memory_write_enable
        //engeliyor çünkü eğer store varsa decode aşamasında memory_write_enable değeri 1 olacak, yoksa 0 olarak kalacağı için aşağıdaki case'lerden herhangi birine girmeyecek. 
        //memory_write_addr değeri ise execute aşamasındaki store komutlarının altındaki kod bloklarında belirleniyor.
        case(instr_d[14:12]) 
         F3_SB: dMem[memory_write_addr[$clog2(MEM_SIZE)-1:0]][7:0]  <= rs2_data[7:0];  //yine $clog2(MEM_SIZE) kullanarak adres değerini [9:0] a düşürdük yani 10 bitlik(maks 1024) olabilecek şekilde ayarladık
         F3_SH: dMem[memory_write_addr[$clog2(MEM_SIZE)-1:0]][15:0] <= rs2_data[15:0]; //çünkü MEM_SIZE değerimiz 1024 yani maksimum 1024 adet adresimiz var [0-1023] arası.
         F3_SW: dMem[memory_write_addr[$clog2(MEM_SIZE)-1:0]]       <= rs2_data;
         default: ;
        endcase
    end
end
//////////////////////////////////////////MEMORY/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////WRITE BACK AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

//write back aşaması, işlemciden çıkan bir sonucu register-file'a yazma işlemi için kullanılır, örnek olarak ADD, SUB, MUL ya da LD, LW, LH gibi işlemler için kullanılır. ADD X3, X2, X1, X2 ve X1 değerini topla
//ve sonucu X3 register'ına yaz şeklinde bir örnek verilebilir. Donanım tarafında şöyle gösterilir, rd_data = x1 + x2, rf[x3] = rd_data.

//Store aşaması ise, işlemcinin mesela bir RAM modülüne(belleğe) veri yazmasını sağlar ve SW,SH,SB gibi komutlar ile kullanılır. SW X5,0(x10) X5 register'ındaki veriyi X10 register'ının gösterdiği bellek adresine
//yaz bu örnekte immediate değeri 0 olarak verilmiş. Donanım tarafında şöyle gösterilir, rs2_data = x5; memory_write_addr = x10 + 0(immediate); dMem[memory_write_addr] <= rs2_data; Şeklinde gösterilir.


    always_ff @(posedge clk_i or negedge rstn_i) begin //register file'a write back aşaması.
        if(!rstn_i) begin
            for(int i = 0; i<32; ++i) begin
                rf[i] <= '0; //eğer reset sinyali aktifse 32 adet register file'ın hepsine 0 değeri atıyoruz.
                end
            end  else if(rf_write_enable && instr_d[11:7] != '0) begin //instr_d[11:7] değeri 0 olmadığı sürece olmasının sebebi RISC-V mimarisinde 0. register her zaman 0 değerini taşır.
                rf[instr_d[11:7]] <= rd_data; //32 adet 32 bitlik register file'ımız olduğu için instr_d[11:7] yani 5 bitlik kısım yetiyor çünkü 2^5 = 32 adet register'a da ulaşmış oluyoruz. rd_data zaten 32 bit.
            end
    end
//////////////////////////////////////////WRITE BACK AŞAMASI//////////////////////////////////////////////////////////////////////////////////////


endmodule
