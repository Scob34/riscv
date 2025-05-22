module fetch
    import riscv_pkg::*;
(
    input  logic            clk_i,                //clock sinyali
    input  logic            rstn_i,               //reset sinyali
    input  logic [XLEN-1:0] pc_i,                 // Fetch aşamasından gelenbir sonraki program counter değeri
    input  logic [XLEN-1:0] instruction_i,        // fetch aşamasındn gelen instruction değeri
    input  logic [     4:0] rd_addr_i,            // Register'a yazılacak verinin hangi adresine yazılacağını ifade eder, decode aşamasından geliyor.
    input  logic            rd_write_enable_i,    // Register'a yazma işlemi yapılıp yapılmayacağını ifade eder, decode aşamasından geliyor.
    input  logic            memory_write_enable_i,// Hafızaya yazma işlemine karar veren sinyal, decode aşamasından geliyor.
    output logic            memory_write_enable_o,// Hafızaya yazma işlemine karar veren sinyali burada execute aşamasına gönderiyoruz.
    output logic [XLEN-1:0] memory_write_addr_o,  // Hafızaya yazacağımız adresi ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic [XLEN-1:0] memory_write_data_o,  // Hafızaya yazacağımız veriyi ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic [XLEN-1:0] rd_data_o,            // Register'a yazılacak veriyi ifade eder ve 
    output logic [     4:0] rd_addr_o,            // Verinin register'ın hangi adresine yazılacağını ifade eder.
    output logic            rd_write_enable_o,    // Register'a yazma işlemi yapılıp yapılmayacağını ifade eder.
    output logic [XLEN-1:0] pc_o,                 // EXECUTE aşamasına gidecek olan pc değeri
    output logic [XLEN-1:0] instruction_o,        // EXECUTE aşamasına gidecek olan instruction değeri
    output logic [XLEN-1:0] alu_data_o,           // ALU'da hesaplanıp ALU'dan çıkan veriyi ifade eder.
    output logic            next_pc_enable_o,     // Branch olacak mı olmayacak mı onu ifade eder. Eğer 1 olursa next_pc pc_q + 4 değil de branch için hesaplanan next_pc_o değeri olur.
    output logic [XLEN-1:0] next_pc_o             // next_pc_enable_o = 1 olduğunda branch için atlanacak olan pc değerini ifade eder.   
);

always_comb begin : execute_block  //
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    next_pc_enable_o     = 0;
    next_pc_o            = 0;
    rd_data_o              = 0;

    //----------------------------------------------------------------------------------------------------------------------------------------- 
    case(instr_d[6:0])
        OpcodeLui: begin
            rd_data_o = imm_data;          //register destination data, yani hedef register verisi imm_data olarak ayarlandı.
        end
        OpcodeAuipc: begin
            rd_data_o = pc_q + imm_data;   //register destination data, yani hedef register verisi imm_data + bir sonraki program counter olarak ayarlandı.  
        end
        OpcodeJal: begin
            next_pc_enable_o = 1'b1;
            next_pc_o = imm_data + pc_q;
            rd_data_o = pc_q + 4;  
        end
        OpcodeJalr: begin
            next_pc_enable_o = 1'b1;
            next_pc_o = imm_data + rs1_data;
            next_pc_o[0] = 1'b0; // Burada en son bit 0 yapılıyor çünkü JALR komutunda en son bitin 0 olması gerekiyor.
            rd_data_o = pc_q + 4; 
        end
        OpcodeBranch:
            case(instr_d[14:12])
             F3_BEQ: if(rs1_data == rs2_data) begin   // BRANCH IF EQUAL
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1;
             end
             F3_BNE: if(rs1_data != rs2_data) begin   // BRANCH IF NOT EQUAL
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1; 
             end 
             F3_BLT: if($signed(rs1_data) < $signed(rs2_data)) begin   // BRANCH IF LESS THAN, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1; 
             end 
             F3_BGE: if($signed(rs1_data) >= $signed(rs2_data)) begin   // BRANCH IF GREATER OR EQUAL, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1; 
             end 
             F3_BLTU: if(rs1_data < rs2_data) begin   // BRANCH IF LESS THAN(UNSIGNED)
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1; 
             end 
             F3_BGEU: if(rs1_data >= rs2_data) begin   // BRANCH IF GREATER OR EQUAL(UNSIGNED)
                next_pc_o = pc_q + imm_data;
                next_pc_enable_o = 1'b1; 
             end
             default: ; 
            endcase
        OpcodeLoad:
            case(instr_d[14:12])
             F3_LB: begin   //LOAD BYTE
                rd_data_o[31:8]   = {24{dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7]}}; //burada ise sign extension yapıyoruz çünkü 32 bitlik rd_data var ve biz alt satırda bu 32 bitlik data içine sadece
                                                                                 //dMem içerisindeki eriştiğimiz adres içindeki verinin son 8 bitini aktardık çünkü Load işlemleri sadece son 8 biti ister.

                rd_data_o[7:0]    = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7:0];     //rs1_data[$clog2[MEM_SIZE]-1:0] bu ifade ile rs1_data[9:0] şeklinde 10 bitlik bir değer gelicek ve bunun sayesinde dMem
                                                                                 //içerisindeki 0-1023 adresten birine erişicez sonrasında bu adres içindeki [7:0] 8 bitlik veriyi rd_data[7:0] a aktarıcaz çünkü
                                                                                 //LB(LOAD BYTE) yapıyoruz yani 1 byte(8 bit) lik bir yükleme yapıyoruz bu nedenle son 8 biti[7:0] çekiyoruz.
             end
             F3_LH: begin  //LOAD HALFBYTE
                rd_data_o[31:16]  = {16{dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15]}};//yine aynı şekilde sign extension yapıyoruz.
                rd_data_o[15:0]   = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                 //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
             end
             F3_LW: begin  //LOAD WORD
                rd_data_o         = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]];          //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LW(LOAD WORD) yapıyoruz yani 4 byte(32 bit)
                                                                                 //yükleme yapıyoruz, zaten 32 bit yüklediğimiz için herhangi bir EXTENSION YAPMAMIZA GEREK KALMIYOR.
             end
             F3_LBU: begin //LOAD BYTE UNSIGNED
                rd_data_o[31:8] = 24'b0;                                           //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli lakin burada LBU(UNSIGNED) olduğu için extension'u
                                                                                 //sign extension yapmıyoruz de ZERO(0) EXTENSION olarak yapıyoruz.
                rd_data_o[7:0]  = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][7:0];
             end
             F3_LHU: begin //LOAD HALFWORD UNSIGNED
                rd_data_o[31:16]  = 16'b0;                                         //Yine SIGN EXTENSION YERINE ZERO EXTENSION YAPTIK, çünkü komut LHU(UNSIGNED) olarak ifade edilmiş gerisi LH ile aynı.
                rd_data_o[15:0]   = dMem[rs1_data[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                   //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
             end
             default: ;
            endcase
        OpcodeStore: ; 
        OpcodeOpImm:
            case(instr_d[14:12])
             F3_ADDI: begin
                rd_data_o = $signed(imm_data) + $signed(rs1_data);
             end
             F3_SLTI: begin  //SET LESS THAN IMMEDIATE
                if($signed(rs1_data) < $signed(imm_data))
                    rd_data_o = 32'b1;
             end
             F3_SLTIU: begin //SET LESS THAN IMMEDIATE(UNSIGNED)
                if(rs1_data < imm_data)
                    rd_data_o = 32'b1;
             end
             F3_XORI: begin
                rd_data_o = rs1_data ^ imm_data;
             end
             F3_ORI: begin
                rd_data_o = rs1_data | imm_data;
             end
             F3_ANDI: begin
                rd_data_o = rs1_data & imm_data;
             end
             F3_SLLI:     //SHIFT LEFT LOGICAL IMMEDIATE
                if(instr_d[31:25] == F7_SLLI) begin
                    rd_data_o = rs1_data << shamt_data;
                end
            F3_SRLI_SRAI:     
                if(instr_d[31:25] == F7_SRLI) begin          //SHIFT RIGHT LOGICAL IMMEDIATE
                    rd_data_o = rs1_data >> shamt_data;
                end else if(instr_d[31:25] == F7_SRAI) begin //SHIFT RIGHT ARITMATIC IMMEDIATE
                    rd_data_o = rs1_data >>> shamt_data;
                end
            default: ;
            endcase
        OpcodeOp:
            case(instr_d[14:12])
             F3_ADD_SUB:
                if(instr_d[31:25] == F7_ADD) begin
                    rd_data_o = rs1_data + rs2_data;
                end else if(instr_d[31:25] == F7_SUB) begin
                    rd_data_o = rs1_data - rs2_data;
                end
            F3_SLL:     begin   //SHIFT LEFT LOGICAL
                rd_data_o = rs1_data << rs2_data;
            end
            F3_SLT:     begin   //SET LESS THAN
                if($signed(rs1_data) < $signed(rs2_data))
                    rd_data_o = 32'b1;
            end
            F3_SLTU:    begin  //SET LESS THAN UNSIGNED
                if(rs1_data < rs2_data)
                    rd_data_o = 32'b1;
            end
            F3_XOR:     begin
                rd_data_o = rs1_data ^ rs2_data;
            end
            F3_SRL_SRA: 
                if(instr_d[31:25] == F7_SRL) begin //SHIFT RIGHT LOGICAL
                    rd_data_o = rs1_data >> rs2_data;
                end else if(instr_d[31:25] == F7_SRA) begin
                    rd_data_o = $signed(rs1_data) >>> rs2_data; //Aritmetik şekilde sağa kaydırmak için hem >>> hem de $signed ifadesi kullanmamız lazım aynı anda
                end
            F3_OR: begin
                rd_data_o = rs1_data | rs2_data;                  
            end
            F3_AND: begin
                rd_data_o = rs1_data & rs2_data;
            end
            default: ;
        endcase
        default: ;
    endcase
end

always_ff @(posedge clk_i or negedge rstn_i) begin
    if(!rstn_i) begin
        pass
    end else begin
        // Store işlemi olması için memory_write_enable sinyalinin 1 olması lazım 1 olmazsa zaten
        // store işlemi gerçekleşmez bu nedenle buradaki verilerin yanlış ayarlanmış olması bir şef ifade etmez, zaten memory_write_enable doğru olduğu zaman
        // buradaki veriler de doğru olmuş olur.
        memory_write_data_o    = rs2_data; // Burada rs2_data değeri store işlemi için hafızaya yazacağımız veriyi ifade eder.
        memory_write_addr_o    = rs1_data + imm_data; // Burada hafızaya yazacağımız adresi hesaplıyoruz. rs1_data + imm_data işlemi ile hafızaya yazacağımız adresi buluyoruz.
    end
    
end

endmodule
