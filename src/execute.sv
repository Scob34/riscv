module execute
    import riscv_pkg::*;
(
    input  logic            clk_i,                //clock sinyali
    input  logic            rstn_i,               //reset sinyali
    input  logic [XLEN-1:0] pc_i,                 // Decode aşamasından gelenbir sonraki program counter değeri
    input  logic [XLEN-1:0] instruction_i,        // Decode aşamasındn gelen instruction değeri
    input  logic [     4:0] rf_addr_i,            // Register'a yazılacak verinin hangi adresine yazılacağını ifade eder, decode aşamasından geliyor.
    input  logic            rf_write_enable_i,    // Register'a yazma işlemi yapılıp yapılmayacağını ifade eder, decode aşamasından geliyor.
    input  logic            memory_write_enable_i,// Hafızaya yazma işlemine karar veren sinyal, decode aşamasından geliyor.
    input  logic [XLEN-1:0] imm_data_i,           // Decode aşamasından gelen immediate değeri
    input  logic [XLEN-1:0] rs1_data_i,           // Decode aşamasından gelen rs1 verisi
    input  logic [XLEN-1:0] rs2_data_i,           // Decode aşamasından gelen rs2 verisi
    input  logic [     4:0] shamt_data_i,         // Shift işlemlerinde kaydırma miktarını ifade eder, decode aşamasından geliyor.
    input  operation_e      operation_i,          // Decode aşamasından gelen işlem türünü ifade eder. Bu işlem türü ALU işlemi, hafıza işlemi veya branch işlemi olabilir.
    output operation_e      operation_o,          // Decode aşamasından gelen işlem türünü ifade eder. Bu işlem türü ALU işlemi, hafıza işlemi veya branch işlemi olabilir.
    output logic            memory_write_enable_o,// Hafızaya yazma işlemine karar veren sinyali burada memory aşamasına gönderiyoruz.
    output logic [XLEN-1:0] memory_write_addr_o,  // Hafızaya yazacağımız adresi ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic [XLEN-1:0] memory_write_data_o,  // Hafızaya yazacağımız veriyi ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic [XLEN-1:0] memory_read_addr_o,   // Hafızadan okuyacağımız adresi ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic            memory_read_enable_o, // Hafızadan okuma işlemi yapılıp yapılmayacağını ifade eder, bu aşamada oluşturup diğer aşamalara gönderiyoruz.
    output logic [XLEN-1:0] alu_data_o,            // Register'a yazılacak veriyi ifade eder ve memory aşamasına gönderiyoruz.
    output logic [     4:0] rf_addr_o,            // Verinin register'ın hangi adresine yazılacağını ifade eder.
    output logic            rf_write_enable_o,    // Register'a yazma işlemi yapılıp yapılmayacağını ifade eder.
    output logic [XLEN-1:0] pc_o,                 // MEMORY aşamasına gidecek olan pc değeri
    output logic [XLEN-1:0] instruction_o,        // MEMORY aşamasına gidecek olan instruction değeri
    output logic            next_pc_enable_o,     // Branch olacak mı olmayacak mı onu ifade eder. Eğer 1 olursa next_pc pc_i + 4 değil de branch için hesaplanan next_pc_d değeri olur.
    output logic [XLEN-1:0] next_pc_o             // next_pc_enable_d = 1 olduğunda branch için atlanacak olan pc değerini ifade eder.   
);

    logic [XLEN-1:0] next_pc_d; // bir sonraki program counter değeri için geçici değişken, sonrasında bu değişkeni flip flop içinde next_pc_d'ya aktaracağız.
    logic            next_pc_enable_d; // bir sonraki program counter'ı güncelleyip güncellemeyeceğimizi ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde next_pc_enable_d'ya aktaracağız.
    logic [XLEN-1:0] alu_data_d; // Register'a yazılacak veriyi ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde alu_data_d'ya aktaracağız.
    logic [XLEN-1:0] memory_write_data_d; // Hafızaya yazacağımız veriyi ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde memory_write_data_d'ya aktaracağız.
    logic [XLEN-1:0] memory_write_addr_d; // Hafızaya yazacağımız adresi ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde memory_write_addr_d'ya aktaracağız.
    logic [XLEN-1:0] memory_read_addr_d;   // Hafızadan okuyacağımız adresi ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde memory_read_addr_o'ya aktaracağız.
    logic            memory_read_enable_d; // Hafızadan okuma işlemi yapılıp yapılmayacağını ifade eden geçici değişken, sonrasında bu değişkeni flip flop içinde memory_read_enable_d'ya aktaracağız.
    logic [XLEN-1:0] memory_read_addr_sum;

always_comb begin : execute_block  //
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    next_pc_enable_d      = 0;
    next_pc_d             = 0;
    alu_data_d            = 0;
    memory_read_addr_d    = 0;
    memory_read_enable_d  = 0;
    memory_read_addr_sum  = 0;
    

    //----------------------------------------------------------------------------------------------------------------------------------------- 
    case(instruction_i[6:0])
        OpcodeLui: begin
            alu_data_d = imm_data_i;          //register destination data, yani hedef register verisi imm_data_i olarak ayarlandı.
        end
        OpcodeAuipc: begin
            alu_data_d = pc_i + imm_data_i;   //register destination data, yani hedef register verisi imm_data_i + bir sonraki program counter olarak ayarlandı.  
        end
        OpcodeJal: begin
            next_pc_enable_d = 1'b1;
            next_pc_d = imm_data_i + pc_i;
            alu_data_d = pc_i + 4;  
        end
        OpcodeJalr: begin
            next_pc_enable_d = 1'b1;
            next_pc_d = imm_data_i + rs1_data_i;
            next_pc_d[0] = 1'b0; // Burada en son bit 0 yapılıyor çünkü JALR komutunda en son bitin 0 olması gerekiyor.
            alu_data_d = pc_i + 4; 
        end
        OpcodeBranch:
            case(instruction_i[14:12])
             F3_BEQ: if(rs1_data_i == rs2_data_i) begin   // BRANCH IF EQUAL
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1;
             end
             F3_BNE: if(rs1_data_i != rs2_data_i) begin   // BRANCH IF NOT EQUAL
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1; 
             end 
             F3_BLT: if($signed(rs1_data_i) < $signed(rs2_data_i)) begin   // BRANCH IF LESS THAN, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1; 
             end 
             F3_BGE: if($signed(rs1_data_i) >= $signed(rs2_data_i)) begin   // BRANCH IF GREATER OR EQUAL, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1; 
             end 
             F3_BLTU: if(rs1_data_i < rs2_data_i) begin   // BRANCH IF LESS THAN(UNSIGNED)
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1; 
             end 
             F3_BGEU: if(rs1_data_i >= rs2_data_i) begin   // BRANCH IF GREATER OR EQUAL(UNSIGNED)
                next_pc_d = pc_i + imm_data_i;
                next_pc_enable_d = 1'b1; 
             end
             default: ; 
            endcase
        OpcodeLoad: begin
            memory_read_addr_sum = rs1_data_i + imm_data_i; //rs1_data_i[$clog2[MEM_SIZE]-1:0] bu ifade ile rs1_data_i[9:0] şeklinde 10 bitlik bir değer gelicek ve bunun sayesinde dMem
            memory_read_addr_d   = memory_read_addr_sum;   //rs1_data_i[$clog2[MEM_SIZE]-1:0] bu ifade ile rs1_data_i[9:0] şeklinde 10 bitlik bir değer gelicek ve bunun sayesinde dMem
                                                                        //içerisindeki 0-1023 adresten birine erişicez sonrasında bu adres içindeki [7:0] 8 bitlik veriyi rd_data[7:0] a aktarıcaz çünkü
                                                                        //LB(LOAD BYTE) yapıyoruz yani 1 byte(8 bit) lik bir yükleme yapıyoruz bu nedenle son 8 biti[7:0] çekiyoruz.
            memory_read_enable_d = 1'b1; // hafızadan okuma işlemi yapacağımızı ifade eden sinyal
        end
        OpcodeStore: ; 
        OpcodeOpImm:
            case(instruction_i[14:12])
             F3_ADDI: begin
                alu_data_d = $signed(imm_data_i) + $signed(rs1_data_i);
             end
             F3_SLTI: begin  //SET LESS THAN IMMEDIATE
                if($signed(rs1_data_i) < $signed(imm_data_i))
                    alu_data_d = 32'b1;
             end
             F3_SLTIU: begin //SET LESS THAN IMMEDIATE(UNSIGNED)
                if(rs1_data_i < imm_data_i)
                    alu_data_d = 32'b1;
             end
             F3_XORI: begin
                alu_data_d = rs1_data_i ^ imm_data_i;
             end
             F3_ORI: begin
                alu_data_d = rs1_data_i | imm_data_i;
             end
             F3_ANDI: begin
                alu_data_d = rs1_data_i & imm_data_i;
             end
             F3_SLLI:     //SHIFT LEFT LOGICAL IMMEDIATE
                if(instruction_i[31:25] == F7_SLLI) begin
                    alu_data_d = rs1_data_i << shamt_data_i;
                end
            F3_SRLI_SRAI:     
                if(instruction_i[31:25] == F7_SRLI) begin          //SHIFT RIGHT LOGICAL IMMEDIATE
                    alu_data_d = rs1_data_i >> shamt_data_i;
                end else if(instruction_i[31:25] == F7_SRAI) begin //SHIFT RIGHT ARITMATIC IMMEDIATE
                    alu_data_d = rs1_data_i >>> shamt_data_i;
                end
            default: ;
            endcase
        OpcodeOp:
            case(instruction_i[14:12])
             F3_ADD_SUB:
                if(instruction_i[31:25] == F7_ADD) begin
                    alu_data_d = rs1_data_i + rs2_data_i;
                end else if(instruction_i[31:25] == F7_SUB) begin
                    alu_data_d = rs1_data_i - rs2_data_i;
                end
            F3_SLL:     begin   //SHIFT LEFT LOGICAL
                alu_data_d = rs1_data_i << rs2_data_i;
            end
            F3_SLT:     begin   //SET LESS THAN
                if($signed(rs1_data_i) < $signed(rs2_data_i))
                    alu_data_d = 32'b1;
            end
            F3_SLTU:    begin  //SET LESS THAN UNSIGNED
                if(rs1_data_i < rs2_data_i)
                    alu_data_d = 32'b1;
            end
            F3_XOR:     begin
                alu_data_d = rs1_data_i ^ rs2_data_i;
            end
            F3_SRL_SRA: 
                if(instruction_i[31:25] == F7_SRL) begin //SHIFT RIGHT LOGICAL
                    alu_data_d = rs1_data_i >> rs2_data_i;
                end else if(instruction_i[31:25] == F7_SRA) begin
                    alu_data_d = $signed(rs1_data_i) >>> rs2_data_i; //Aritmetik şekilde sağa kaydırmak için hem >>> hem de $signed ifadesi kullanmamız lazım aynı anda
                end
            F3_OR: begin
                alu_data_d = rs1_data_i | rs2_data_i;                  
            end
            F3_AND: begin
                alu_data_d = rs1_data_i & rs2_data_i;
            end
            default: ;
        endcase
        default: ;
    endcase
end

always_ff @(posedge clk_i or negedge rstn_i) begin
    if(!rstn_i) begin
        // Burada reset sinyali 0 olursa alu_data_o, next_pc_enable_o, next_pc_o, pc_o, instruction_o, rf_addr_o, rf_write_enable_o değerlerini sıfırlıyoruz.
        alu_data_o             <= 32'b0; 
        next_pc_enable_o       <= 1'b0; 
        next_pc_o              <= 32'b0; 
        pc_o                   <= 32'b0; 
        instruction_o          <= 32'b0; 
        rf_addr_o              <= 5'b0; 
        rf_write_enable_o      <= 1'b0; 
        memory_write_enable_o  <= 1'b0;
        operation_o            <= ZERO; // Burada operation_o'yu sıfırlıyoruz çünkü işlem türünü ifade ediyor.
        memory_read_addr_o     <= 32'b0;
        memory_read_enable_o   <= 1'b0; 
    end else begin
        // Burada ALU'dan gelen veriyi ALU'dan çıkışa aktarıyoruz.
        alu_data_o             <= alu_data_d; // ALU'dan gelen veriyi alu_data_o'ya aktarıyoruz.
        next_pc_enable_o       <= next_pc_enable_d; // ALU'dan gelen next_pc_enable_d değerini dışarıya aktarıyoruz.
        next_pc_o              <= next_pc_d; // ALU'dan gelen next_pc_d değerini dışarıya aktarıyoruz.
        pc_o                   <= pc_i; // Program counter'ı dışarıya aktarıyoruz.
        instruction_o          <= instruction_i; // Instruction'ı dışarıya aktarıyoruz.
        rf_addr_o              <= rf_addr_i; // Register'a yazılacak verinin hangi adresine yazılacağını ifade eden değeri dışarıya aktarıyoruz.
        rf_write_enable_o      <= rf_write_enable_i; // Register'a yazma işlemi yapılıp yapılmayacağını ifade eden değeri dışarıya aktarıyoruz.
        memory_write_enable_o  <= memory_write_enable_i; // Hafızaya yazma işlemi yapılıp yapılmayacağını ifade eden değeri dışarıya aktarıyoruz. 
        memory_write_data_o    <= memory_write_data_d; // Hafızaya yazacağımız veriyi dışarıya aktarıyoruz.
        memory_write_addr_o    <= memory_write_addr_d; // Hafızaya yazacağımız adresi dışarıya aktarıyoruz.
        operation_o            <= operation_i; // İşlem türünü dışarıya aktarıyoruz.
        memory_read_addr_o     <= memory_read_addr_d; // Hafızadan okuyacağımız adresi dışarıya aktarıyoruz.
        memory_read_enable_o   <= memory_read_enable_d; // Hafızadan okuma işlemi yapılıp yapılmayacağını ifade eden değeri dışarıya aktarıyoruz.
    end

end

//Bu aşamada hafızaya yazacağımız veriyi ve adresi belirliyoruz, memory aşamasına göndericez.
always_comb begin
        // Store işlemi olması için memory_write_enable sinyalinin 1 olması lazım 1 olmazsa zaten
        // store işlemi gerçekleşmez bu nedenle buradaki verilerin yanlış ayarlanmış olması bir şef ifade etmez, zaten memory_write_enable doğru olduğu zaman
        // buradaki veriler de doğru olmuş olur.
        memory_write_data_d    = rs2_data_i; // Burada rs2_data_i değeri store işlemi için hafızaya yazacağımız veriyi ifade eder.
        memory_write_addr_d    = rs1_data_i + imm_data_i; // Burada hafızaya yazacağımız adresi hesaplıyoruz. rs1_data_i + imm_data_i işlemi ile hafızaya yazacağımız adresi buluyoruz.
end

endmodule
