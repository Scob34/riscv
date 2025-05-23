module decode
    import riscv_pkg::*;
(
    input  logic            clk_i,                //clock sinyali
    input  logic            rstn_i,               //reset sinyali
    input  logic [XLEN-1:0] pc_i,                 // Fetch aşamasından gelenbir sonraki program counter değeri
    input  logic [XLEN-1:0] instruction_i,        // fetch aşamasındn gelen instruction değeri
    input  logic [XLEN-1:0] rf_data_i,            // Register'a yazılacak veriyi ifade eder ama dışarıdan gelen bir datadır.(Bu yapıda memory aşamasından gelcek)
    input  logic [     4:0] rf_addr_i,            // Verinin register'ın hangi adresine yazılacağını ifade eder ama dışarıdan gelen bir adrestir.
    input  logic            rf_write_enable_i,    // Register'a yazma işlemi yapılıp yapılmayacağını ifade eder ama dışarıdan gelen bir sinyaldir.
    output logic [XLEN-1:0] pc_o,                 // EXECUTE aşamasına gidecek olan pc değeri
    output logic [XLEN-1:0] instruction_o,        // EXECUTE aşamasına gidecek olan instruction değeri
    output operation_e      operation_o,          // Decode aşamasında opcode'ye göre belirlenen işlemi ifade eder. Bu işlem Execute aşamasındaki ALU'ya gönderilir.
    output logic [XLEN-1:0] rs1_data_o,           // Register'dan okunan rs1 verisini dışarıy verebilmemiz için gerekli olan port.
    output logic [XLEN-1:0] rs2_data_o,           // Register'dan okunan rs2 verisini dışarıy verebilmemiz için gerekli olan port.
    output logic [     4:0] rf_addr_o,            // Verinin register'ın hangi adresine yazılacağını dışarıya vermek için gerekli olan port.
    output logic            rf_write_enable_o,    // Register'a yazma işlemi yapılıp yapılmayacağını dışarıya vermek için gerekli olan port.
    output logic            memory_write_enable_o,// Belleğe yazma işlemi yapılıp yapılmayacağını dışarıya vermek için gerekli olan port.
    output logic [XLEN-1:0] imm_o,                // Decode bloğunda oluşturulan immediate değerini Execute aşamasına göndermek için gerekli olan port.
    output logic [     4:0] shamt_data_o          // Decode bloğunda oluşturulan shamt değerini Execute aşamasına göndermek için gerekli olan port.
);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    logic [XLEN-1:0] rs1_data;                    //Register Source 1. İlk kaynak(source) register'ıdır.
    logic [XLEN-1:0] rs2_data;                    //Register Source 2. İkinci kaynak register'ıdır.
    logic [XLEN-1:0] imm_data;                    //İşlemciye register'dan değil de direkt verilen datayı ifade eder. immediate data
    logic [4:0]      shamt_data;                  //Bazı komutlarda kullandığımız shamt için kullandığımız datayı ifade eder.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    logic [XLEN-1:0] rf [31:0];         // 32 adet 32 bit register file'ı tanımlıyoruz. RISC-V mimarisinde register file'ı 32 adet 32 bit register'dan oluşur.
    logic            rf_write_enable;   //register file write enable. Eğer bu 1 ise register file içindeki ilgili adrese(register destination) rd_data(register_destination data) yazılır.
                                        // rf_write_enable rf_write_enable_i ye nazaran dışarıdan gelen bir sinyal değil, içerideki bir sinyali ifade eder fark budur
                                        // yoksa ikisi de register file'a yazma işlemi yapılıp yapılmayacağını kontrol eder.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    operation_e      operation_d;  // Decode aşamasında opcode'ye göre belirlenen işlemi ifade eder. Burada enum içindeki her değere decode_block içinde
                              // bir opcode atayacağız o yüzden operation_d oluşturduk. Sonrasında bunları operation_o ile çıkışa vericez.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    logic            memory_write_enable_d; // Belleğe yazma işlemi yapılıp yapılmayacağını ifade eder. Belleğe yazma işlemi yapılıp yapılmayacağını ifade eden sinyali dışarıya vermek için gerekli olan port.
always_comb begin : decode_block   //instr_d içindeki instruction'un decode edilmesi, bu aşamada instr_d'nin belirli kısımlarındaki OPCODE'ların ne olduğuna göre case'ler açıp o case'ler içinde o OPCODE'nin görevi
                                   //neyse onları yapıcaz. Opcode dediğimiz kısım istr_d'nin son 7 biti yani instr_d[6:0] kısmı.
    
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    imm_data              = 32'b0;
    shamt_data            = 5'b0;
    rs1_data              = 32'b0;
    rs2_data              = 32'b0;
    rf_write_enable       = 1'b0;
    memory_write_enable_d = 1'b0;
    //---------------------------------------------------------------------------------------------------------------------------------------
    case(instruction_i[6:0])
        OpcodeLui: begin                //Lui = Load Upper Immediate
            operation_d = LUI;  // operation_e adlı enum içerisindeki LUI değerini operation_d'ya atıyoruz ki Execute aşamasında bu opcode'ye göre işlem yapabilelim. Dİğerleri için de aynı şey geçerli.
            imm_data = {instruction_i[31:12], 12'b0};  //LUI komutunun tanımına göre imm_data'ya atama yaptık. Tanımda instruction'un 31:12 bitini al ve geri kalanına 0 ata diyor o nedenle biz de öyle yaptık.
            rf_write_enable = 1'b1; //LUI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
        end
        OpcodeAuipc: begin
            operation_d = AUIPC;
            imm_data = {instruction_i[31:12], 12'b0};
            rf_write_enable = 1'b1; //AUIPC komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
        end   
        OpcodeJal: begin
            operation_d = JAL; //JAL = Jump and Link
            imm_data[31:21] = {11{instruction_i[31]}};
            imm_data[20]    = instruction_i[31];
            imm_data[10:1]  = instruction_i[30:21];
            imm_data[11]    = instruction_i[20];
            imm_data[19:12] = instruction_i[19:12];
            imm_data[0]     = 1'b0;
            rf_write_enable = 1'b1; //JAL komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz. 
        end
        OpcodeJalr:
            if(instruction_i[14:12] == F3_JALR) begin
                operation_d = JALR; //JALR = Jump and Link Register
                rs1_data = rf[instruction_i[19:15]];
                imm_data[31:12] = {20{instruction_i[31]}};
                imm_data[11:0]  = instruction_i[31:20];
                rf_write_enable = 1'b1; //JALR komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
            end
        

        OpcodeBranch: begin   // Branch işlemlerinde rs1, rs2 ve imm adresleri tüm branch işlemlerinde aynı olduğu için bu şekilde tek seferde yazdık.
            rs1_data        = rf[instruction_i[19:15]];
            rs2_data        = rf[instruction_i[24:20]];
            imm_data[31:13] = {19{instruction_i[31]}};   // riscv instruction pdf'indeki talimatlara göre IMM DATA'NIN HANGİ BİTİNİN HANGİ INSTRUCTION BITLERİNE KARŞILIK GELDİĞİNİ belirtiyoruz.
            imm_data[12]    = instruction_i[31];
            imm_data[11]    = instruction_i[7];
            imm_data[10:5]  = instruction_i[30:25];
            imm_data[4:1]   = instruction_i[11:8];
            imm_data[0]     = 1'b0;
                             
            // Branch işlemlerinde farklı olan şey EXECUTE aşamasında hangi işlemin yapılacağıdır. Bunun için de FUNCT3 koduna bakıyoruz ve ona göre operation_d ataması yapıyoruz.
            case(instruction_i[14:12])          //FUNCT3 koduna bakarak EXECUTE aşamasında hangi işlemi yapacağımıza karar veriyoruz.
                F3_BEQ:  operation_d = BEQ;     //BEQ = Branch if Equal
                F3_BNE:  operation_d = BNE;     //BNE = Branch if Not Equal
                F3_BLT:  operation_d = BLT;     //BLT = Branch if Less Than
                F3_BGE:  operation_d = BGE;     //BGE = Branch if Greater Than or Equal
                F3_BLTU: operation_d = BLTU;    //BLTU = Branch if Less Than Unsigned
                F3_BGEU: operation_d = BGEU;    //BGEU = Branch if Greater Than or Equal Unsigned
                default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
            endcase
        end
        OpcodeLoad: begin                  //aynı şekilde opcodebranch'daki olayın aynısını burada yapıyoruz.
            rs1_data        = rf[instruction_i[19:15]];
            imm_data[31:12] = {20{instruction_i[31]}};
            imm_data[11:0]  = instruction_i[31:20];

            case(instruction_i[14:12])          //FUNCT3 koduna bakıyoruz
                F3_LB:  begin   
                    operation_d = LB;      //LB = Load Byte
                    rf_write_enable = 1'b1; //LB komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.(WRITE BACK İÇİN)
                end
                F3_LH:  begin   
                    operation_d = LH;      //LH = Load Halfword
                    rf_write_enable = 1'b1; //LH komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                end
                F3_LW:  begin   
                    operation_d = LW;      //LW = Load Word
                    rf_write_enable = 1'b1; //LW komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                end
                F3_LBU: begin  
                    operation_d = LBU;     //LBU = Load Byte Unsigned
                    rf_write_enable = 1'b1; //LBU komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                end
                F3_LHU: begin  
                    operation_d = LHU;     //LHU = Load Halfword Unsigned
                    rf_write_enable = 1'b1; //LHU komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                end
                default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
                
                //rf_write_enable değerini her case'de ayrı ayrı 1 yapmamızın sebebi, eğer hiçbir case girmeyip operation_d UNKNOWN olursa rf_write_enable 0 
                //olarak kalmalı bu nedenle çünkü diğer türlü en başta rs1 ve imm ile beraber yaparsak UNKKOWN olduğunda bile rf_write_enable 1 olur
                //bu durum istemediğimiz bir durum.
            endcase
        end
        OpcodeStore: begin
            rs1_data        = rf[instruction_i[19:15]];
            rs2_data        = rf[instruction_i[24:20]];
            imm_data[31:12] = {20{instruction_i[31]}};
            imm_data[11:5]  = instruction_i[31:25];
            imm_data[4:0]   = instruction_i[11:7];

            case(instruction_i[14:12])          //FUNCT3 koduna bakıyoruz
                F3_SB: begin   
                    operation_d = SB;      //SB = Store Byte
                    memory_write_enable_d = 1'b1; //SB komutunda belleğe yazma işlemi yapılacağı için memory_write_enable'ı 1 yapıyoruz.
                end
                F3_SH: begin   
                    operation_d = SH;      //SH = Store Halfword
                    memory_write_enable_d = 1'b1; //SH komutunda belleğe yazma işlemi yapılacağı için memory_write_enable'ı 1 yapıyoruz.
                end
                F3_SW: begin   
                    operation_d = SW;      //SW = Store Word
                    memory_write_enable_d = 1'b1; //SW komutunda belleğe yazma işlemi yapılacağı için memory_write_enable'ı 1 yapıyoruz.
                end
                default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
            endcase
        end
        OpcodeOpImm:
            case(instruction_i[14:12])          //FUNCT3 koduna bakıyoruz
            //ADD IMMEDIATE, sonunda I varsa Immediate, yoksa mesela F3_ADD ise normal ADD işlemi. Diğer kodlar için de böyle.
                F3_ADDI, F3_SLTI, F3_SLTIU, F3_XORI, F3_ORI, F3_ANDI : begin
                    rs1_data        = rf[instruction_i[19:15]];
                    imm_data[31:12] = {20{instruction_i[31]}};
                    imm_data[11:0]  = instruction_i[31:20];
                    
                    case(instruction_i[14:12])
                        F3_ADDI:  begin  
                            operation_d = ADDI;    //ADDI = Add Immediate
                            rf_write_enable = 1'b1; //ADDI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_SLTI:  begin  
                            operation_d = SLTI;    //SLTI = Set Less Than Immediate
                            rf_write_enable = 1'b1; //SLTI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_SLTIU: begin 
                            operation_d = SLTIU;   //SLTIU = Set Less Than Immediate Unsigned
                            rf_write_enable = 1'b1; //SLTIU komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_XORI:  begin  
                            operation_d = XORI;    //XORI = Exclusive OR Immediate
                            rf_write_enable = 1'b1; //XORI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_ORI:   begin   
                            operation_d = ORI;     //ORI = OR Immediate
                            rf_write_enable = 1'b1; //ORI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_ANDI:  begin  
                            operation_d = ANDI;    //ANDI = AND Immediate
                            rf_write_enable = 1'b1; //ANDI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        default:  operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
                    endcase 
                end
                F3_SLLI :
                    if(instruction_i[31:25] == F7_SLLI) begin
                        operation_d = SLLI; //SLLI = Shift Left Logical Immediate
                        shamt_data = instruction_i[24:20];
                        rs1_data   = rf[instruction_i[19:15]];
                        rf_write_enable = 1'b1; //SLLI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.          
                    end
                F3_SRLI_SRAI :
                    if(instruction_i[31:25] == F7_SRLI) begin
                        operation_d = SRLI; //SRLI = Shift Right Logical Immediate
                        shamt_data = instruction_i[24:20];
                        rs1_data   = rf[instruction_i[19:15]];
                        rf_write_enable = 1'b1; //SRLI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                    end
                    else if(instruction_i[31:25] == F7_SRAI) begin
                        operation_d = SRAI; //SRAI = Shift Right Arithmetic Immediate
                        shamt_data = instruction_i[24:20];
                        rs1_data   = rf[instruction_i[19:15]];
                        rf_write_enable = 1'b1; //SRAI komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                    end
                default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
            endcase
        OpcodeOp:
            case(instruction_i[14:12])
                F3_ADD_SUB :
                    if(instruction_i[31:25] == F7_ADD) begin
                        operation_d = ADD;
                        rs1_data   = rf[instruction_i[19:15]];
                        rs2_data   = rf[instruction_i[24:20]];
                        rf_write_enable = 1'b1; //ADD komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                    end else if(instruction_i[31:25] == F7_SUB) begin
                        operation_d = SUB;
                        rs1_data   = rf[instruction_i[19:15]];
                        rs2_data   = rf[instruction_i[24:20]];
                        rf_write_enable = 1'b1; //SUB komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                    end
                F3_SLL, F3_SLT, F3_SLTU, F3_XOR, F3_OR, F3_AND : begin
                    rs1_data   = rf[instruction_i[19:15]];
                    rs2_data   = rf[instruction_i[24:20]];
                    case(instruction_i[14:12])
                        F3_SLL:  begin  
                            operation_d = SLL;     //SLL = Shift Left Logical
                            rf_write_enable = 1'b1; //SLL komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_SLT:  begin  
                            operation_d = SLT;     //SLT = Set Less Than
                            rf_write_enable = 1'b1; //SLT komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_SLTU: begin 
                            operation_d = SLTU;    //SLTU = Set Less Than Unsigned
                            rf_write_enable = 1'b1; //SLTU komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_XOR:  begin  
                            operation_d = XOR;     //XOR = Exclusive OR
                            rf_write_enable = 1'b1; //XOR komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_OR:   begin   
                            operation_d = OR;      //OR = OR
                            rf_write_enable = 1'b1; //OR komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        F3_AND:  begin  
                            operation_d = AND;     //AND = AND
                            rf_write_enable = 1'b1; //AND komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                        end
                        default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
                    endcase
                end
                F3_SRL_SRA :
                    if(instruction_i[31:25] == F7_SRL) begin
                        operation_d = SRL; //SRL = Shift Right Logical
                        rs1_data   = rf[instruction_i[19:15]];
                        rs2_data   = rf[instruction_i[24:20]];
                        rf_write_enable = 1'b1; //SRL komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.                            
                    end else if(instruction_i[31:25] == F7_SRA) begin
                        operation_d = SRA; //SRA = Shift Right Arithmetic
                        rs1_data   = rf[instruction_i[19:15]];
                        rs2_data   = rf[instruction_i[24:20]];
                        rf_write_enable = 1'b1; //SRA komutunda register'a yazma işlemi yapıldığı için rf_write_enable'ı 1 yapıyoruz.
                    end
                default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
            endcase
        default: operation_d = UNKNOWN; //herhangi farklı bir sinyal gelirse waveformdan görmek için UNKNOWN atıyoruz.
    endcase
end

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

always_ff @(posedge clk_i or negedge rstn_i) begin
    if(!rstn_i) begin
        pc_o                  <= 32'b0;
        instruction_o         <= 32'b0;
        rs1_data_o            <= 32'b0;
        rs2_data_o            <= 32'b0;
        imm_o                 <= 32'b0;
        rs1_data_o            <= 32'b0;
        rs2_data_o            <= 32'b0;
        memory_write_enable_o <= 1'b0;
        operation_o           <= ZERO; //operation_o'yu sıfırlıyoruz çünkü operation_d'dan gelen değerler sıfırlanmadığı için burada sıfırlıyoruz.
        shamt_data_o          <= 5'b0;
        rf_addr_o             <= 5'b0;
        rf_write_enable_o     <= 1'b0;
    
    end else begin
        pc_o                  <= pc_i;                //Fetch aşamasından gelen pc değerini pc_o ile Execute aşamasına gönderiyoruz değiştirmeden.
        instruction_o         <= instruction_i; //Fetch aşamasından gelen instruction değerini instruction_o ile Execute aşamasına gönderiyoruz değiştirmeden.
        rs1_data_o            <= rs1_data;            //Decode aşamasında rs1_data'ya atadığımız değeri rs1_data_o ile Execute aşamasına gönderiyoruz.
        rs2_data_o            <= rs2_data;            //Decode aşamasında rs2_data'ya atadığımız değeri rs2_data_o ile Execute aşamasına gönderiyoruz.
        imm_o                 <= imm_data;            //Decode aşamasında imm_data'ya atadığımız değeri imm_o ile Execute aşamasına gönderiyoruz.
        memory_write_enable_o <= memory_write_enable_d; //Belleğe yazma işlemi yapılıp yapılmayacağını ifade eden sinyali dışarıya veriyoruz.
        operation_o           <= operation_d;         //Decode aşamasında belirlediğimiz işlemi dışarıya veriyoruz.
        shamt_data_o          <= shamt_data;          //Decode aşamasında shamt_data'ya atadığımız değeri shamt_data_o ile Execute aşamasına gönderiyoruz.
        rs1_data_o            <= rs1_data;            //Decode aşamasında rs1_data'ya atadığımız değeri rs1_data_o ile Execute aşamasına gönderiyoruz.
        rs2_data_o            <= rs2_data;            //Decode aşamasında rs2_data'ya atadığımız değeri rs2_data_o ile Execute aşamasına gönderiyoruz.
        rf_addr_o             <= instruction_i[11:7]; //register file'a yazılacak olan adresi instruction'un 11:7 bitlerinden alıyoruz. Bu kısım opcode'ye göre değişiyor. Burada opcode'ye göre değişen kısım instruction'un 11:7 bitleri. (WRITE BACK AŞAMASI İÇİN)
        rf_write_enable_o     <= rf_write_enable;     //register file'a yazma işlemi yapılıp yapılmayacağını ifade eden sinyali dışarıya veriyoruz. (WRITE BACK AŞAMASI İÇİN)
    end
    
end


///////////////////////////////////////// WRITE BACK AŞAMASI ////////////////////////////////////////////////////////////////////////////////////////////////
//write back aşaması, işlemciden çıkan bir sonucu register-file'a yazma işlemi için kullanılır, örnek olarak ADD, SUB, MUL ya da LD, LW, LH gibi işlemler için kullanılır. ADD X3, X2, X1, X2 ve X1 değerini topla
//ve sonucu X3 register'ına yaz şeklinde bir örnek verilebilir. Donanım tarafında şöyle gösterilir, rd_data = x1 + x2, rf[x3] = rd_data.

// WRITE BACK aşamasında dışarıdan gelen veriler rd_data, rd_addr ve rd_write_enable kullanılır.

//Store aşaması ise, işlemcinin mesela bir RAM modülüne(belleğe) veri yazmasını sağlar ve SW,SH,SB gibi komutlar ile kullanılır. SW X5,0(x10) X5 register'ındaki veriyi X10 register'ının gösterdiği bellek adresine
//yaz bu örnekte immediate değeri 0 olarak verilmiş. Donanım tarafında şöyle gösterilir, rs2_data = x5; memory_write_addr = x10 + 0(immediate); dMem[memory_write_addr] <= rs2_data; Şeklinde gösterilir.


    always_ff @(posedge clk_i or negedge rstn_i) begin //register file'a yazma işlemi
        if(!rstn_i)
            for(int i = 0; i<32; ++i) begin
                rf[i] <= '0; //eğer reset sinyali aktifse 32 adet register file'ın hepsine 0 değeri atıyoruz.
                end
        else if(rf_write_enable_i && rf_addr_i != '0) //instr_d[11:7] değeri 0 olmadığı sürece olmasının sebebi RISC-V mimarisinde 0. register her zaman 0 değerini taşır.
                rf[rf_addr_i] <= rf_data_i; //32 adet 32 bitlik register file'ımız olduğu için instr_d[11:7] yani 5 bitlik kısım yetiyor çünkü 2^5 = 32 adet register'a da ulaşmış oluyoruz. rd_data zaten 32 bit.
    end
//WRITE BACK aşamasını ayrı bir modül olarak yazmak yerine decode block içinde yazdık çünkü küçük bir kod bloğu olduğu için ayrı bir modül yazmaya gerek yok.
////////////////////////////////////////// WRITE BACK AŞAMASI ///////////////////////////////////////////////////////////////////////////////////////////////
endmodule
