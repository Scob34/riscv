module core_model
import riscv_pkg::*;
(input  logic clk_i, rstn_i);          //instructor memory için CLK ve RST


//XLEN ifadesi işlemcinin veriyolu genişliğini yani kaç bit olduğunu ifade ediyor. Burada XLEN 32 olduğu için 32 bitlik bir işlemciden bahsediyoruz.


logic [XLEN-1:0] jump_pc_d; /* başka modüllerden program counter'ını başka bir değere ani olarak atlatmak için komut geldiğinde o modüller aynı zamanda bir program_counter(pc)
de gönderecek o zaman o gelen değişken bu jump_pc_d değişkenine aktarılıp bunun üzerinden kullanılacak.*/
logic            jump_pc_valid_d; // başka bir modülden komut gelip gelmediğini tutan kontrol değişkeni. Eğer komut geldiyse 1 olur gelmediyse 0 da kalır.
logic [XLEN-1:0] instr_d;  //giriş instruction(talimat/komut). Zaten instruction bir girdi olduğu için _d ile ifade edilir. Kısaca giriş kabloları _d, çıkış kabloları _q ile
//ifade edilir.



endmodule
