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
logic [31:0]     dMem [MEM_SIZE-1:0]; // data memory tanımlamasını yaptık

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



logic [XLEN-1:0] jump_pc_d; /* başka modüllerden program counter'ını başka bir değere ani olarak atlatmak için komut geldiğinde o modüller aynı zamanda bir program_counter(pc)
de gönderecek o zaman o gelen değişken bu jump_pc_d değişkenine aktarılıp bunun üzerinden kullanılacak.*/
logic            jump_pc_valid_d; // başka bir modülden komut gelip gelmediğini tutan kontrol değişkeni. Eğer komut geldiyse 1 olur gelmediyse 0 da kalır.
logic [XLEN-1:0] instr_d;  //giriş instruction(talimat/komut). Zaten instruction bir girdi olduğu için _d ile ifade edilir. Kısaca giriş kabloları _d, çıkış kabloları _q ile
//ifade edilir.

assign data_o     = dMem[addr_i];
assign instr_o    = instr_d;  //instr_d = iMem[pc_d[$clog2(MEM_SIZE) - 1 : 0]]; aşağıdaki programCounter_change_comb kısmında yandaki komut ile instr_d'yi okuduk zaten ve direkt instr_o'ya aktardık ki çıkışta görelim
assign reg_addr_o = rf_write_enable ? instr_d[11:7] : '0;
assign reg_data_o = rd_data;

logic [XLEN-1:0] rd_data;      //Register Destination. İşlemin sonucunun yazılacağı register'dır.  Mesela, ADD rd,rs1,rs2 bu işlemde rs1 register'ı içindeki veri ile rs2 register'ı içindeki veriyi topla rd registerı
                               //içine yaz demek oluyor. ADD rd,rs1,10 ifadesindeki 10 değeri imm_data'yı ifade eder.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
logic [XLEN-1:0] memory_write_data;     // Store aşamasında hafızaya yazacağımız veriyi ifade eder.
logic [XLEN-1:0] memory_write_addr; //Belleğin hangi adresine yazacaksak o adresi hesaplamak için gerekli değişken.
logic            memory_write_enable; // Hafızaya yazabilmemiz için hafızanın write enable'ının 1 olması gerekir.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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




endmodule
