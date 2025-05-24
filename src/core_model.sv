module core_model
import riscv_pkg::*;
 (
    input  logic clk_i, 
    input  logic rstn_i,
    output logic [XLEN-1:0] pc_o,          //program counter
    output logic [XLEN-1:0] instr_o,       //instruction
    output logic [     4:0] reg_addr_o,     //register address
    output logic [XLEN-1:0] reg_data_o,     //register data
    output logic            reg_update_o    //register update
 );          //instructor memory için CLK ve RST

    /////////////////////////////FETCH TO DECODE//////////////////////////////////////
    logic [XLEN-1:0] pc_fetch_to_decode;
    logic [XLEN-1:0] instruction_fetch_to_decode;
    
    /////////////////////////////DECODE TO EXECUTE////////////////////////////////////
    logic [XLEN-1:0] pc_decode_to_execute;
    logic [XLEN-1:0] instruction_decode_to_execute;
    operation_e      operation_decode_to_execute;
    logic [XLEN-1:0] rs1_data_decode_to_execute;
    logic [XLEN-1:0] rs2_data_decode_to_execute;
    logic [     4:0] rf_addr_decode_to_execute;
    logic            rf_write_enable_decode_to_execute;
    logic            memory_write_enable_decode_to_execute;
    logic [XLEN-1:0] immediate_decode_to_execute;
    logic [     4:0] shamt_decode_to_execute;
    
    /////////////////////////////EXECUTE TO MEMORY/////////////////////////////////////
    operation_e      operation_execute_to_memory;
    logic            memory_write_enable_execute_to_memory;
    logic [XLEN-1:0] memory_write_addr_execute_to_memory;
    logic [XLEN-1:0] memory_write_data_execute_to_memory;
    logic [XLEN-1:0] memory_read_addr_execute_to_memory;
    logic            memory_read_enable_execute_to_memory;
    logic [XLEN-1:0] alu_data_o_execute_to_memory;
    logic [     4:0] rf_addr_execute_to_memory;
    logic            rf_write_enable_execute_to_memory;
    logic [XLEN-1:0] pc_execute_to_memory;
    logic [XLEN-1:0] instruction_execute_to_memory;
    logic            next_pc_enable_execute_to_memory;
    logic [XLEN-1:0] next_pc_execute_to_memory;

    /////////////////////////////MEMORY TO DECODE//////////////////////////////////
    logic [XLEN-1:0] memory_read_data_memory_to_decode;
    logic [     4:0] rf_addr_memory_to_decode;
    logic            rf_write_enable_memory_to_decode;
    logic [XLEN-1:0] rf_write_data_memory_to_decode;

    fetch u_fetch (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .next_pc_i(next_pc_execute_to_memory),
        .next_pc_enable_i(next_pc_enable_execute_to_memory),
        .pc_o(pc_fetch_to_decode),
        .instr_o(instruction_fetch_to_decode)
    );

    decode u_decode (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .pc_i(pc_fetch_to_decode),
        .instruction_i(instruction_fetch_to_decode),
        .rf_data_i(rf_write_data_memory_to_decode),
        .rf_addr_i(rf_addr_memory_to_decode),
        .rf_write_enable_i(rf_write_enable_memory_to_decode),
        .pc_o(pc_decode_to_execute),
        .instruction_o(instruction_decode_to_execute),
        .operation_o(operation_decode_to_execute),
        .rs1_data_o(rs1_data_decode_to_execute),
        .rs2_data_o(rs2_data_decode_to_execute),
        .rf_addr_o(rf_addr_decode_to_execute),
        .rf_write_enable_o(rf_write_enable_decode_to_execute),
        .memory_write_enable_o(memory_write_enable_decode_to_execute),
        .imm_o(immediate_decode_to_execute),
        .shamt_data_o(shamt_decode_to_execute)
    );

    execute u_execute (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .pc_i(pc_decode_to_execute),
        .pc_o(pc_execute_to_memory),
        .instruction_i(instruction_decode_to_execute),
        .instruction_o(instruction_execute_to_memory),
        .next_pc_enable_o(next_pc_enable_execute_to_memory),
        .next_pc_o(next_pc_execute_to_memory),
        .operation_i(operation_decode_to_execute),
        .rs1_data_i(rs1_data_decode_to_execute),
        .rs2_data_i(rs2_data_decode_to_execute),
        .rf_addr_i(rf_addr_decode_to_execute),
        .rf_write_enable_i(rf_write_enable_decode_to_execute),
        .memory_write_enable_i(memory_write_enable_decode_to_execute),
        .imm_data_i(immediate_decode_to_execute),
        .shamt_data_i(shamt_decode_to_execute),
        .alu_data_o(alu_data_o_execute_to_memory),
        .memory_write_addr_o(memory_write_addr_execute_to_memory),
        .memory_write_data_o(memory_write_data_execute_to_memory),
        .memory_write_enable_o(memory_write_enable_execute_to_memory),
        .memory_read_addr_o(memory_read_addr_execute_to_memory),
        .memory_read_enable_o(memory_read_enable_execute_to_memory),
        .operation_o(operation_execute_to_memory),
        .rf_addr_o(rf_addr_execute_to_memory),
        .rf_write_enable_o(rf_write_enable_execute_to_memory)
    );

    memory u_memory(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .memory_write_enable_i(memory_write_enable_execute_to_memory),
        .memory_write_addr_i(memory_write_addr_execute_to_memory),
        .memory_write_data_i(memory_write_data_execute_to_memory),
        .operation_i(operation_execute_to_memory),
        .rf_addr_i(rf_addr_execute_to_memory),
        .rf_write_enable_i(rf_write_enable_execute_to_memory),
        .memory_read_addr_i(memory_read_addr_execute_to_memory),
        .memory_read_enable_i(memory_read_enable_execute_to_memory),
        .rf_addr_o(rf_addr_memory_to_decode),
        .rf_write_enable_o(rf_write_enable_memory_to_decode),
        .memory_read_data_o(memory_read_data_memory_to_decode),
        .rf_data_i(alu_data_o_execute_to_memory),
        .rf_data_o(rf_write_data_memory_to_decode)
    );

    assign pc_o = pc_execute_to_memory;
    assign instr_o = instruction_execute_to_memory;
    assign reg_addr_o = rf_addr_execute_to_memory;
    assign reg_data_o = rf_write_data_memory_to_decode;
    assign reg_update_o = rf_write_enable_execute_to_memory;






endmodule
