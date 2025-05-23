module fetch
    import riscv_pkg::*;
(
    input  logic            clk_i,               //clock sinyali
    input  logic            rstn_i,              //reset sinyali
    input  logic [XLEN-1:0] next_pc_i,           //bir sonraki program counter değeri
    input  logic            next_pc_enable_i,    //bir sonraki program counter ı güncellemeye izin verip vermediğini belirten sinyal
    output logic [XLEN-1:0] pc_o,                //program counter çıkış değeri, program counter'ı dışarıya verebilmek için.
    output logic [XLEN-1:0] instr_o            //instruction çıkış değeri, instruction'ı dışarıya verebilmek için.
);

//parameter int MEM_SIZE = 2048; // Memory tanımlamalarında bu parametreyi kullanacağız.
logic [31:0] iMem [MEM_SIZE-1:0]; //instruction memory, instruction'ları saklamak için kullandığımız bellek
initial $readmemh("./test/test.hex", iMem, 0, MEM_SIZE);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//program counter + instruction tanımlamaları
logic [XLEN-1:0] pc_d;   // program counter'ın girişi. Yani sıradaki program counter değerini ifade eder.
logic [XLEN-1:0] pc_q;   // program counter'ın çıkışı. Yani şu anki kullanılan program counter değerini ifade eder.
assign pc_o      = pc_q; // O anki program counter değerini pc_o ile dışarıya veriyoruz.

always_ff @(posedge clk_i or negedge rstn_i) begin  : programCounter_change_flipFlop  // program counter'ın değiştiği blok
    if(!rstn_i) begin
        pc_q <= 'h8000_0000; // eğer reset sinyali 0 ise program counter'ına 0 değeri atanır yani program counter'ı resetlenir. PC'yi 8000_0000 yapmamızın sebebi
                             // SPIKE SIMULATORU'nun RISC-V'da program counter'ı 0x8000_0000 adresinden başlatmasıdır.
    end
    else begin
        pc_q <= pc_d;       // eğer reset inyali 1 ise program counter'ına sıradaki program counter'ı atanır,
    end

end

always_comb begin : programCounter_change_comb
    pc_d = pc_q;
    if(next_pc_enable_i)        // eğer next_pc_enable_i değeri 1 olduysa dışarıdan komut gelmiştir ve başka bir program counter(pc) gelmiştir o halde işlemci counter'ını normal
        pc_d = next_pc_i;      // şekilde işletmek yerine dışarıdan gelen bu program counter'ı pc_d girişine atıyoruz ve bu sayede yeni program counter'ı çıkışa aktarılıyor 
    else                       // yukarıdaki pc_q <= pc_d sayesinde
        pc_d = pc_q + 4;       //burada ise yeni program counter eski program counter'ın üstüne + 4 eklenmiş hali olarak devam ediyor.
    
    instr_o = iMem[pc_q[$clog2(MEM_SIZE*4) - 1 : 2]]; /* Burada pc_q[$clog2(MEM_SIZE-1):2] ifadesi şunun için var, pc_q yukarıda [XLEN-1:0] şeklinde tanımlandı yani 32 bit, fakat bizim Memory Size'ımız 1024 yani
    adresleme yapmak için 32 bite değil yalnızca 10 bite ihtiyacımız var bu nedenle $clog2 fonksiyonunu kullanarak pc_q'yi 32 bitten 10 bite indirgiyoruz. Bu nasıl oluyor $clog2(parametre) fonksiyonu
    bir sayının 2 tabanındaki logaritmasını hesaplar, bu nedenle $clog2(MEM_SIZE-1) ifadesi MEM_SIZE değeri 1024 olduğu için 1024-1 = 1023 sayısının 2 tabanındaki logaritmasını hesaplar bu da 10'a eşittir.
    Bu durumda pc_q[$clog2(MEM_SIZE-1) - 1 : 0] ifadesi aslında pc_d[(10-1):0] => pc_q[9:0] şekline indirgenmiş oluyor yani pc_q(sıradaki program counter) değerinin SON 10 BİTİNE indirgenmiş olduk.*/

    /*NOT: instr_o = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 0]]; değil de instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; yapmamızın sebebi iMem'in logic [31:0]     iMem [MEM_SIZE-1:0]; şeklinde WORD ADRESLEME
    yapmasından kaynaklı yani iMem[0] = {byte3, byte2, byte1, byte0}, iMem[1] = {byte7,byte6,byte5,byte4} gibi. instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; risc-v'da bu şekilde alt iki biti çıkardığımız zaman 
    BYTE ADRESLEME MODUNDAN WORD ADRESLEME MODUNA GEÇİYORUZ. Eğer bunu yapmazsak mesela adres 00000004 olduğunda iMem[1]'e erişmesi gerekirken iMem[4]'e erişir bu da yanlıştır çünkü 00000004 ifadesi 4. byte'ı
    yani iMem[1]'in başlangıç adresini ifade eder.*/

    /* MEM_SIZE'ı 4 ile çarpmamızın sebebi, MEM_SIZE bize aslında KAÇ ADET WORD olduğunu gösteriyor yani 1024 adet WORD'umuz var fakat bir WORD İÇİNDE 4 BYTE OLDUĞU İÇİN  MEM_SIZE * 4 yapıyoruz ve belleğin
    toplam boyutunu BYTE CİNSİNDEN HESAPLIYORUZ Kİ PROGRAM COUNTER BYTE BYTE İLERLEDİĞİ İÇİN TOPLAM BYTE ALANINI HESAPLAMIŞ OLALIM.*/
end
endmodule 
