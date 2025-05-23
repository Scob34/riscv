module memory
    import riscv_pkg::*;
(
    input  logic            clk_i,                 //clock sinyali
    input  logic            rstn_i,                //reset sinyali
    input  logic            memory_write_enable_i, // hafızaya yazma işlemi için gerekli olan sinyal,
    input  logic [XLEN-1:0] memory_write_addr_i,   // hafızaya yazacağımız adres
    input  logic [XLEN-1:0] memory_write_data_i,   // hafızaya yazacağımız veri
    input  operation_e      operation_i,           // hangi store işlemini yapacağımızı belirten sinyal
    input  logic [     4:0] rf_addr_i,             // register file'dan aldığımız adres, execute aşamasından geldi WRITE BACK IÇIN KULLANILACAK
    input  logic            memory_read_enable_i,  // hafızadan okuma işlemi için gerekli olan sinyal
    input  logic [XLEN-1:0] memory_read_addr_i,    // hafızadan okuyacağımız adres
    output logic [XLEN-1:0] memory_read_data_o,    // hafızadan okuduğumuz veri
    output logic [     4:0] rf_addr_o             // register file'a yazacağımız adres (WRITE BACK AŞAMASI İÇİN DECODE BLOĞUNA GÖNDERİLECEK.)
    );

//memory modülünde hafızaya veri yazıyoruz(store işlemi).

logic [31:0]     dMem [MEM_SIZE-1:0]; // data memory tanımlamasını yaptık

always_ff @(posedge clk_i or negedge rstn_i) begin   //Store işlemi için gerekli kod bloğu
    if(!rstn_i) begin
    end
    else if(memory_write_enable_i) begin
        //sanki instr_d[14:12] değeri aşağıdaki işlemlerden birine denk gelirse store işlemi olsa da olmasa da aşağıdaki işlemlerden biri gerçekleşiyor gibi gözüküyor fakat bu durumu memory_write_enable
        //engeliyor çünkü eğer store varsa decode aşamasında memory_write_enable değeri 1 olacak, yoksa 0 olarak kalacağı için aşağıdaki case'lerden herhangi birine girmeyecek. 
        //memory_write_addr değeri ise execute aşamasındaki store komutlarının altındaki kod bloklarında belirleniyor.
        case(operation_i) //operation_i değeri decode aşamasında belirleniyor. Burada hangi store işleminin yapılacağını belirliyoruz.
         SB: dMem[memory_write_addr_i[$clog2(MEM_SIZE)-1:0]][7:0]  <= memory_write_data_i[7:0];  //yine $clog2(MEM_SIZE) kullanarak adres değerini [9:0] a düşürdük yani 10 bitlik(maks 1024) olabilecek şekilde ayarladık
         SH: dMem[memory_write_addr_i[$clog2(MEM_SIZE)-1:0]][15:0] <= memory_write_data_i[15:0]; //çünkü MEM_SIZE değerimiz 1024 yani maksimum 1024 adet adresimiz var [0-1023] arası.
         SW: dMem[memory_write_addr_i[$clog2(MEM_SIZE)-1:0]]       <= memory_write_data_i;
         default: ;
        endcase
    end
    rf_addr_o <= rf_addr_i; //register file'a yazacağımız adresi belirliyoruz. Burada rf_addr_i değeri decode aşamasında belirleniyor.
end

always_comb begin
    if(memory_read_enable_i) begin
        case(operation_i)
            LB: begin
                memory_read_data_o[31:8]  = {24{dMem[memory_read_addr_i][7]}}; //LB(LOAD BYTE) yaptığımız için alt 8 bitin üstüne sign extension yapıyoruz.
                memory_read_data_o[7:0]   = dMem[memory_read_addr_i][7:0]; //LB(LOAD BYTE) yaptığımız için alt 8 bitlik veriyi alıyoruz.
            end  
            LH: begin
                memory_read_data_o[31:16] = {16{dMem[memory_read_addr_i][15]}}; //LH(LOAD HALFWORD) yaptığımız için alt 16 bitin üstüne sign extension yapıyoruz.
                memory_read_data_o[15:0]  = dMem[memory_read_addr_i][15:0];     //LH(LOAD HALFWORD) yaptığımız için alt 16 bitlik veriyi alıyoruz.
            end 
            LW: begin 
                memory_read_data_o = dMem[memory_read_addr_i];                //direkt 32 bit olduğu için direkt atıyoruz.
            end
            LBU: begin
                memory_read_data_o[31:8]  = 24'b0;
                memory_read_data_o[7:0]   = dMem[memory_read_addr_i][7:0];    //sign extension yerine zero extension yapıyoruz çünkü LBU(UNSIGNED) olduğu için.
            end
            LHU: begin
                memory_read_data_o[31:16] = 16'b0;
                memory_read_data_o[15:0]  = dMem[memory_read_addr_i][15:0];   //sign extension yerine zero extension yapıyoruz çünkü LHU(UNSIGNED) olduğu için.
            end
            default: ;
        endcase
    end
end

endmodule
