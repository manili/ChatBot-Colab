#include <iostream>

#define CACHE_SIZE          0xFF + 1

#define LD_ST_NONE          0x00
#define LD_ST_WORD          0x01
#define LD_ST_BYTE_SIGNED   0x02
#define LD_ST_HALF_SIGNED   0x03
#define LD_ST_BYTE_UNSIGNED 0x04
#define LD_ST_HALF_UNSIGNED 0x05

#define RF_WR_SRC_TYP_MEM   0x00
#define RF_WR_SRC_TYP_ALU   0x01

#define JMP_NONE            0x00
#define JMP_BRCH            0x01
#define JMP_JALR            0x02
#define JMP_JAL             0x03

#define ALU_NOP             0x00
#define ALU_ADD             0x01
#define ALU_AND             0x02
#define ALU_BEQ             0x03
#define ALU_BGE             0x04
#define ALU_BGEU            0x05
#define ALU_BLT             0x06
#define ALU_BLTU            0x07
#define ALU_BNE             0x08
#define ALU_OR              0x09
#define ALU_SLL             0x0A
#define ALU_SLT             0x0B
#define ALU_SLTU            0x0C
#define ALU_SRA             0x0D
#define ALU_SRL             0x0E
#define ALU_SUB             0x0F
#define ALU_XOR             0x10

class CPU {
public:
    // Simulation related signals
    bool startup; // Processor is  starting up
    uint64_t tick; // Clock cycle counter

    // Internal registers
    uint32_t pc; // program counter
    uint32_t rf[32]; // 32 registers of 32-bit

    // Caches
    uint32_t i_cache[CACHE_SIZE];
    uint32_t d_cache[CACHE_SIZE];

    // Reset all class variables to default values
    void reset();

    // Run one cycle of processor (execute one instruction)
    void run_one_tick();

    // Class constructor
    CPU();

private:
    struct FetcherSignals {
        // Fetched instruction
        uint32_t instruction;
    };

    struct DecoderSignals {
        // Immediate Value
        int32_t imm;

        // ALU Inputs
        uint8_t alu_op;
        int32_t alu_in1;
        int32_t alu_in2;

        // Register-File read data and write-enable signals
        int32_t rf_rd_dat1;
        int32_t rf_rd_dat2;
        uint32_t rf_wr_addr;
        uint8_t rf_wr_src_type;
        bool rf_wr_en;

        // D-Cache write-enable signal
        uint8_t load_store_type;
        bool dcache_wr_en;

        // Branch, Jump type
        uint8_t jump_type;

        // Is instruction valid
        bool is_valid;
    };

    struct AluSignals {
        // Inputs
        uint8_t operation;
        int32_t operand1;
        int32_t operand2;

        // Outputs
        int32_t result;
        bool overflow;
    };

    struct MemorySignals {
        uint32_t mem_rd_dat;
    };

    // Golobal Signals
    FetcherSignals fetcher_signals;
    DecoderSignals decoder_signals;
    AluSignals alu_signals;
    MemorySignals memory_signals;

    // Functions for different stages of the datapath
    void fetch();       // Stage 1 - Instruction Fetch
    void decode();      // Stage 2 - Instruction Decode
    void execute();     // Stage 3 - Instruction Execute
    void memory();      // Stage 4 - Instruction Memory
    void write_back();  // Stage 5 - Instruction Write Back

    // Helper functions
    void alu();
    void i_to_b(uint32_t in, uint8_t *out);
    int32_t sign_extend(uint32_t imm, uint32_t size);

    uint32_t get_opcode(uint32_t instr);
    uint32_t get_funct3(uint32_t instr);
    uint32_t get_funct7(uint32_t instr);
    uint32_t get_rs1(uint32_t instr);
    uint32_t get_rs2(uint32_t instr);
    uint32_t get_rd(uint32_t instr);
    int32_t get_imm_I(uint32_t instr);
    int32_t get_imm_S(uint32_t instr);
    int32_t get_imm_B(uint32_t instr);
    int32_t get_imm_U(uint32_t instr);
    int32_t get_imm_J(uint32_t instr);
};

CPU::CPU() {
    // set startup flag to 1
    startup = 1;
    
    // set clock cycle counter to 0
    tick = 0;

    // set program counter to 0
    pc = 0;

    // initialize all registers to zero
    for (int i = 0; i < 32; i++) {
        rf[i] = 0;
    }

    // initialize all caches to zero
    for (int i = 0; i < CACHE_SIZE; i++) {
        i_cache[i] = 0;
        d_cache[i] = 0;
    }
}

void CPU::reset() {
    // Reset CPU Internal Variables
    startup = 1;
    pc = 0;

    // Reset Fetcher Signals
    fetcher_signals.instruction = 0x00000000;

    // Reset Decoder Signals
    decoder_signals.imm = 0;
    decoder_signals.alu_in1 = 0;
    decoder_signals.alu_in2 = 0;
    decoder_signals.alu_op = ALU_NOP;
    decoder_signals.rf_rd_dat1 = 0;
    decoder_signals.rf_rd_dat2 = 0;
    decoder_signals.rf_wr_addr = 0;
    decoder_signals.rf_wr_src_type = RF_WR_SRC_TYP_ALU; 
    decoder_signals.rf_wr_en = false;
    decoder_signals.load_store_type = LD_ST_NONE;
    decoder_signals.dcache_wr_en = false;
    decoder_signals.jump_type = JMP_NONE;
    decoder_signals.is_valid = true;

    // Reset Alu Signals
    alu_signals.operation = ALU_NOP;
    alu_signals.operand1 = 0;
    alu_signals.operand2 = 0;
    alu_signals.result = 0;
    alu_signals. overflow = false;

    // Reset Memory Signals
    memory_signals.mem_rd_dat = 0;
}

void CPU::run_one_tick() {
    fetch();
    decode();
    execute();
    memory();
    write_back();

    tick++;
}

void CPU::fetch() {
    // Update next PC address respect to ALU and Decode results.
    int32_t jmp_amount = 0;

    if (startup == 1) {
        startup = 0;
        jmp_amount = 0;
    } else if (decoder_signals.jump_type != JMP_NONE) {
        if (decoder_signals.jump_type == JMP_JAL ||
            (decoder_signals.jump_type == JMP_BRCH && alu_signals.result == 1)) {
            jmp_amount = decoder_signals.imm;
        } else if (decoder_signals.jump_type == JMP_JALR) {
            jmp_amount = decoder_signals.rf_rd_dat1 + decoder_signals.imm;
        } else {
            jmp_amount = 4;
        }
    } else {
        jmp_amount = 4;
    }

    if (jmp_amount % 4 != 0)
        std::cerr << "Error: Next PC address is not a valid address and caused misaligned exception." << std::endl;

    pc = pc + jmp_amount;
    fetcher_signals.instruction = i_cache[pc];
}

void CPU::decode() {
    uint32_t opcode = get_opcode(fetcher_signals.instruction);
    uint32_t funct3 = get_funct3(fetcher_signals.instruction);
    uint32_t funct7 = get_funct7(fetcher_signals.instruction);
    uint32_t rs1 = get_rs1(fetcher_signals.instruction);
    uint32_t rs2 = get_rs2(fetcher_signals.instruction);
    uint32_t rd = get_rd(fetcher_signals.instruction);
    int32_t imm_i = get_imm_I(fetcher_signals.instruction);
    int32_t imm_s = get_imm_S(fetcher_signals.instruction);
    int32_t imm_b = get_imm_B(fetcher_signals.instruction);
    int32_t imm_u = get_imm_U(fetcher_signals.instruction);
    int32_t imm_j = get_imm_J(fetcher_signals.instruction);

    decoder_signals.imm = 0;
    decoder_signals.alu_in1 = 0;
    decoder_signals.alu_in2 = 0;
    decoder_signals.alu_op = ALU_NOP;
    decoder_signals.rf_rd_dat1 = rf[rs1];
    decoder_signals.rf_rd_dat2 = rf[rs2];
    decoder_signals.rf_wr_addr = rd;
    decoder_signals.rf_wr_src_type = RF_WR_SRC_TYP_ALU; 
    decoder_signals.rf_wr_en = false;
    decoder_signals.load_store_type = LD_ST_NONE;
    decoder_signals.dcache_wr_en = false;
    decoder_signals.jump_type = JMP_NONE;
    decoder_signals.is_valid = true;

    switch (opcode) {
        case 0b0110111: // LUI
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.imm = imm_u;
            decoder_signals.alu_in1 = decoder_signals.imm;
            decoder_signals.rf_wr_en = true;
            break;
        case 0b0010111: // AUIPC
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.alu_in1 = pc;
            decoder_signals.imm = imm_u;
            decoder_signals.alu_in2 = decoder_signals.imm;
            decoder_signals.rf_wr_en = true;
            break;
        case 0b1101111: // JAL
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.alu_in1 = pc;
            decoder_signals.imm = imm_j;
            decoder_signals.rf_wr_en = true;
            decoder_signals.jump_type = JMP_JAL;
            break;
        case 0b1100111: // JALR
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.alu_in1 = pc;
            decoder_signals.imm = imm_i;
            decoder_signals.rf_wr_en = true;
            decoder_signals.jump_type = JMP_JALR;
            break;
        case 0b1100011: // Branch instructions
            decoder_signals.alu_in1 = decoder_signals.rf_rd_dat1;
            decoder_signals.alu_in2 = decoder_signals.rf_rd_dat2;
            decoder_signals.imm = imm_b;
            decoder_signals.jump_type = JMP_BRCH;
            switch (funct3) {
                case 0b000: // BEQ
                    decoder_signals.alu_op = ALU_BEQ;
                    break;
                case 0b001: // BNE
                    decoder_signals.alu_op = ALU_BNE;
                    break;
                case 0b100: // BLT
                    decoder_signals.alu_op = ALU_BLT;
                    break;
                case 0b101: // BGE
                    decoder_signals.alu_op = ALU_BGE;
                    break;
                case 0b110: // BLTU
                    decoder_signals.alu_op = ALU_BLTU;
                    break;
                case 0b111: // BGEU
                    decoder_signals.alu_op = ALU_BGEU;
                    break;
                default:
                    std::cerr << "Error: unsupported Branch instruction function code: " << funct3 << std::endl;
                    decoder_signals.is_valid = false;
                    break;
            }
            break;
        case 0b0000011: // Load instructions
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.imm = imm_i;
            decoder_signals.alu_in1 = decoder_signals.rf_rd_dat1;
            decoder_signals.alu_in2 = decoder_signals.imm;
            decoder_signals.rf_wr_en = true;
            decoder_signals.rf_wr_src_type = RF_WR_SRC_TYP_MEM;
            switch (funct3) {
                case 0b000: // LB: Load Byte then Sign-extend (signed)
                    decoder_signals.load_store_type = LD_ST_BYTE_SIGNED;
                    break;
                case 0b001: // LH: Load Half then Sign-extend (signed)
                    decoder_signals.load_store_type = LD_ST_HALF_SIGNED;
                    break;
                case 0b010: // LW: Load Word
                    decoder_signals.load_store_type = LD_ST_WORD;
                    break;
                case 0b100: // LBU: Load Byte then zero-extend (unsigned)
                    decoder_signals.load_store_type = LD_ST_BYTE_UNSIGNED;
                    break;
                case 0b101: // LHU: Load Half then zero-extend (unsigned)
                    decoder_signals.load_store_type = LD_ST_HALF_UNSIGNED;
                    break;
                default:
                    std::cerr << "Error: unsupported Load instruction function code: " << funct3 << std::endl;
                    decoder_signals.is_valid = false;
                    break;
            }
            break;
        case 0b0100011: // Store instructions
            decoder_signals.alu_op = ALU_ADD;
            decoder_signals.alu_in1 = decoder_signals.rf_rd_dat1;
            decoder_signals.alu_in2 = imm_s;
            decoder_signals.dcache_wr_en = true;
            switch (funct3) {
                case 0b000: // SB: Store Byte
                    decoder_signals.load_store_type = LD_ST_BYTE_UNSIGNED;
                    break;
                case 0b001: // SH: Store Half
                    decoder_signals.load_store_type = LD_ST_HALF_UNSIGNED;
                    break;
                case 0b010: // SW: Store Word
                    decoder_signals.load_store_type = LD_ST_WORD;
                    break;
                default:
                    std::cerr << "Error: unsupported Store instruction function code: " << funct3 << std::endl;
                    decoder_signals.is_valid = false;
                    break;
            }
            break;
        case 0b0010011: // Register-Immidiate instructions
            decoder_signals.rf_wr_en = true;
            decoder_signals.alu_in1 = decoder_signals.rf_rd_dat1;
            decoder_signals.imm = imm_i;
            decoder_signals.alu_in2 = decoder_signals.imm;
            switch(funct3) {
                case 0b000: // ADDI
                    decoder_signals.alu_op = ALU_ADD;
                    decoder_signals.imm = sign_extend(imm_i, 12);
                    decoder_signals.alu_in2 = decoder_signals.imm;
                    break;
                case 0b010: // SLTI
                    decoder_signals.alu_op = ALU_SLT;
                    decoder_signals.imm = sign_extend(imm_i, 12);
                    decoder_signals.alu_in2 = decoder_signals.imm;
                    break;
                case 0b011: // SLTIU
                    decoder_signals.alu_op = ALU_SLTU;
                    decoder_signals.imm = sign_extend(imm_i, 12);
                    decoder_signals.alu_in2 = decoder_signals.imm;
                    break;
                case 0b100: // XORI
                    decoder_signals.alu_op = ALU_XOR;
                    break;
                case 0b110: // ORI
                    decoder_signals.alu_op = ALU_OR;
                    break;
                case 0b111: // ANDI
                    decoder_signals.alu_op = ALU_AND;
                    break;
                case 0b001: // SLLI
                    decoder_signals.alu_op = ALU_SLL;
                    break;
                case 0b101: // SRLI or SRAI
                    if (funct7 == 0b0000000) { // SRLI
                        decoder_signals.alu_op = ALU_SRL;
                    } else if (funct7 == 0b0100000) { // SRAI
                        decoder_signals.alu_op = ALU_SRA;
                    } else {
                        std::cerr << "Error: unsupported SRLI or SRAI instruction function code: " << funct7 << std::endl;
                        decoder_signals.is_valid = false;
                    }
                    break;
                default:
                    std::cerr << "Error: unsupported Register-Immidiate instruction function code: " << funct3 << std::endl;
                    decoder_signals.is_valid = false;
                    break;
            }
            break;
        case 0b0110011: // Register-Register instructions
            decoder_signals.rf_wr_en = true;
            decoder_signals.alu_in1 = decoder_signals.rf_rd_dat1;
            decoder_signals.alu_in2 = decoder_signals.rf_rd_dat2;
            switch (funct3) {
                case 0b000: // ADD or SUB
                    if (funct7 == 0b0000000) { // ADD
                        decoder_signals.alu_op = ALU_ADD;
                    } else if (funct7 == 0b0100000) { // SUB
                        decoder_signals.alu_op = ALU_SUB;
                    } else {
                        std::cerr << "Error: unsupported Register-Register instruction function code: " << funct7 << std::endl;
                        decoder_signals.is_valid = false;
                    }
                    break;
                case 0b001: // SLL
                    decoder_signals.alu_op = ALU_SLL;
                    break;
                case 0b010: // SLT
                    decoder_signals.alu_op = ALU_SLT;
                    break;
                case 0b011: // SLTU
                    decoder_signals.alu_op = ALU_SLTU;
                    break;
                case 0b100: // XOR
                    decoder_signals.alu_op = ALU_XOR;
                    break;
                case 0b101: // SRL or SRA
                    if (funct7 == 0b0000000) { // SRL
                        decoder_signals.alu_op = ALU_SRL;
                    } else if (funct7 == 0b0100000) { // SRA
                        decoder_signals.alu_op = ALU_SRA;
                    } else {
                        std::cerr << "Error: unsupported Register-Register instruction function code: " << funct7 << std::endl;
                        decoder_signals.is_valid = false;
                    }
                    break;
                case 0b110: // OR
                    decoder_signals.alu_op = ALU_OR;
                    break;
                case 0b111: // AND
                    decoder_signals.alu_op = ALU_AND;
                    break;
                default:
                    std::cerr << "Error: unsupported Register-Register instruction function code: " << funct3 << std::endl;
                    decoder_signals.is_valid = false;
                    break;
            }
            break;
        case 0b1110011: // System instructions
            std::cerr << "Error: System instructions are not supported, yet!" << std::endl;
            decoder_signals.is_valid = false;
            break;
        default:
            std::cerr << "Error: unsupported instruction: " << fetcher_signals.instruction << std::endl;
            decoder_signals.is_valid = false;
            break;
    }
}

void CPU::execute() {
    alu_signals.operation = decoder_signals.alu_op;
    alu_signals.operand1 = decoder_signals.alu_in1;
    alu_signals.operand2 = decoder_signals.alu_in2;
    alu();
}

void CPU::memory() {
    if (decoder_signals.load_store_type == LD_ST_NONE) return;
    
    if (decoder_signals.dcache_wr_en == true) {
        int32_t write_data_tmp = decoder_signals.rf_rd_dat2;
        switch (decoder_signals.load_store_type) {
            case LD_ST_BYTE_UNSIGNED:
                write_data_tmp = write_data_tmp & 0x000000FF;
                break;
            case LD_ST_HALF_UNSIGNED:
                write_data_tmp = write_data_tmp & 0x0000FFFF;
                break;
            case LD_ST_WORD:
                write_data_tmp = write_data_tmp;
                break;
            default:
                std::cerr << "Error: memory write type is not valid. ";
                break;
        }
        d_cache[alu_signals.result] = write_data_tmp;
    } else {
        int32_t read_data_tmp = d_cache[alu_signals.result];
        switch (decoder_signals.load_store_type) {
            case LD_ST_BYTE_SIGNED:
                read_data_tmp = read_data_tmp & 0x000000FF;
                read_data_tmp = (read_data_tmp << 24) >> 24;
                break;
            case LD_ST_HALF_SIGNED:
                read_data_tmp = read_data_tmp & 0x0000FFFF;
                read_data_tmp = (read_data_tmp << 16) >> 16;
                break;
            case LD_ST_BYTE_UNSIGNED:
                read_data_tmp = read_data_tmp & 0x000000FF;
                break;
            case LD_ST_HALF_UNSIGNED:
                read_data_tmp = read_data_tmp & 0x0000FFFF;
                break;
            case LD_ST_WORD:
                read_data_tmp = read_data_tmp;
                break;
            default:
                std::cerr << "Error: memory read type is not valid. ";
                break;
        }
        memory_signals.mem_rd_dat = read_data_tmp;
    }
}

void CPU::write_back() {
    if (decoder_signals.is_valid == true && decoder_signals.rf_wr_en == true && decoder_signals.rf_wr_addr != 0) {
        if (decoder_signals.rf_wr_src_type == RF_WR_SRC_TYP_MEM) {
            rf[decoder_signals.rf_wr_addr] = memory_signals.mem_rd_dat;
        } else if (decoder_signals.rf_wr_src_type == RF_WR_SRC_TYP_ALU) {
            rf[decoder_signals.rf_wr_addr] = alu_signals.result;
        }
    }
}

void CPU::alu() {
    switch (alu_signals.operation) {
        case ALU_NOP:
            alu_signals.result = 0;
            break;
        case ALU_ADD:
            alu_signals.result = alu_signals.operand1 + alu_signals.operand2;
            alu_signals.overflow = ((alu_signals.operand1 ^ alu_signals.result) & (alu_signals.operand2 ^ alu_signals.result)) >> 31;
            break;
        case ALU_AND:
            alu_signals.result = alu_signals.operand1 & alu_signals.operand2;
            break;
        case ALU_BEQ:
            alu_signals.result = alu_signals.operand1 == alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_BGE:
            alu_signals.result = alu_signals.operand1 >= alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_BGEU:
            alu_signals.result = (uint32_t)alu_signals.operand1 >= (uint32_t)alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_BLT:
            alu_signals.result = alu_signals.operand1 < alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_BLTU:
            alu_signals.result = (uint32_t)alu_signals.operand1 < (uint32_t)alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_BNE:
            alu_signals.result = alu_signals.operand1 != alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_OR:
            alu_signals.result = alu_signals.operand1 | alu_signals.operand2;
            break;
        case ALU_SLL:
            alu_signals.result = alu_signals.operand1 << (alu_signals.operand2 & 0x1F);
            break;
        case ALU_SLT:
            alu_signals.result = alu_signals.operand1 < alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_SLTU:
            alu_signals.result = (uint32_t)alu_signals.operand1 < (uint32_t)alu_signals.operand2 ? 1 : 0;
            break;
        case ALU_SRA:
            alu_signals.result = alu_signals.operand1 >> (alu_signals.operand2 & 0x1F);
            break;
        case ALU_SRL:
            alu_signals.result = (uint32_t)alu_signals.operand1 >> (alu_signals.operand2 & 0x1F);
            break;
        case ALU_SUB:
            alu_signals.result = alu_signals.operand1 - alu_signals.operand2;
            alu_signals.overflow = ((alu_signals.operand1 ^ alu_signals.operand2) & (alu_signals.operand1 ^ alu_signals.result)) >> 31;
            break;
        case ALU_XOR:
            alu_signals.result = alu_signals.operand1 ^ alu_signals.operand2;
            break;
        default:
            std::cerr << "Error: unsupported ALU operation code: " << alu_signals.operation << std::endl;
            break;
    }
}

// Takes an immediate value and its size in bits and sign extends it to a 32-bit signed integer.
int32_t CPU::sign_extend(uint32_t imm, uint32_t size) {
    uint32_t sign_bit = 1 << (size - 1);
    int32_t sign_extend_mask = -1 * ((imm & sign_bit) >> (size - 1));
    return (imm | (sign_extend_mask << size));
}

// Returns the opcode of a given instruction (bits 0-6)
uint32_t CPU::get_opcode(uint32_t instr) {
    return instr & 0x7F;
}

// Returns the funct3 field of a given instruction (bits 12-14)
uint32_t CPU::get_funct3(uint32_t instr) {
    return (instr >> 12) & 0x7;
}

// Returns the funct7 field of a given instruction (bits 25-31)
uint32_t CPU::get_funct7(uint32_t instr) {
    return (instr >> 25) & 0x7F;
}

// Returns the rs1 field of a given instruction (bits 15-19)
uint32_t CPU::get_rs1(uint32_t instr) {
    return (instr >> 15) & 0x1F;
}

// Returns the rs2 field of a given instruction (bits 20-24)
uint32_t CPU::get_rs2(uint32_t instr) {
    return (instr >> 20) & 0x1F;
}

// Returns the rd field of a given instruction (bits 7-11)
uint32_t CPU::get_rd(uint32_t instr) {
    return (instr >> 7) & 0x1F;
}

// Returns the immediate field of an I-type instruction (bits 20-31, sign-extended)
int32_t CPU::get_imm_I(uint32_t instr) {
    return (int32_t)(instr >> 20);
}

// Returns the immediate field of an S-type instruction (bits 7-11 and 25-31, sign-extended and concatenated)
int32_t CPU::get_imm_S(uint32_t instr) {
    uint32_t imm_11_5 = (instr >> 25) & 0x7F;
    uint32_t imm_4_0 = (instr >> 7) & 0x1F;
    uint32_t imm = (imm_11_5 << 5) | imm_4_0;
    return (int32_t)(imm << 20) >> 20; // sign-extend to 32 bits
}

// Returns the immediate field of a B-type instruction (bits 8-11, 25-30, and 7, sign-extended and concatenated)
int32_t CPU::get_imm_B(uint32_t instr) {
    uint32_t imm_11 = (instr >> 7) & 0x1;
    uint32_t imm_10_5 = (instr >> 25) & 0x3F;
    uint32_t imm_4_1 = (instr >> 8) & 0xF;
    uint32_t imm_12 = (instr >> 31) & 0x1;
    uint32_t imm = (imm_12 << 12) | (imm_11 << 11) | (imm_10_5 << 5) | (imm_4_1 << 1);
    return (int32_t)(imm << 19) >> 19; // sign-extend to 32 bits
}

// Returns the immediate field of a U-type instruction (bits 12-31, zero-extended and left-shifted by 12 bits)
int32_t CPU::get_imm_U(uint32_t instr) {
    return instr & 0xFFFFF000;
}

// Returns the immediate field of a J-type instruction (bits 12-19, 20, 21-30, and 31, sign-extended and concatenated)
int32_t CPU::get_imm_J(uint32_t instr) {
    uint32_t imm_20 = (instr >> 31) & 0x1;
    uint32_t imm_19_12 = (instr >> 12) & 0xFF;
    uint32_t imm_11 = (instr >> 20) & 0x1;
    uint32_t imm_10_1 = (instr >> 21) & 0x3FF;
    uint32_t imm = (imm_20 << 20) | (imm_19_12 << 12) | (imm_11 << 11) | (imm_10_1 << 1);
    return (int32_t)(imm << 11) >> 11; // sign-extend to 32 bits
}

void CPU::i_to_b(uint32_t in, uint8_t *out) {
    for (int i = 31; i > 1; i--) {
        out[i] = in % 2 + '0';
        in = in / 2;
    }
    out[1] = in % 2 + '0';
    out[0] = in / 2 + '0';
}

void test1_simple_test(CPU &cpu) {
//=============================================================================================================//
//==================================================================== intr   rd      ,   rs1     ,   rs2    ==//
//=============================================================================================================//
    cpu.i_cache[0x00000000] = 0b00000000000100000000000010010011; //== addi   x1      ,   x0      ,   0x1    ==//
    cpu.i_cache[0x00000004] = 0b00000000001000000000000100010011; //== addi   x2      ,   x0      ,   0x2    ==//
    cpu.i_cache[0x00000008] = 0b00000000001000001000000100110011; //== add    x2      ,   x1      ,   x2     ==//
    cpu.i_cache[0x0000000c] = 0b01101111111000000000000110010011; //== addi   x3      ,   x0      ,   0x0EFE ==//
    cpu.i_cache[0x00000010] = 0b00000000001100000000000000100011; //== sb     x0(0)   ,   x3                 ==//
    cpu.i_cache[0x00000014] = 0b00000000000000000000001000000011; //== lb     x4      ,   x0(0)              ==//
    cpu.i_cache[0x00000018] = 0b00000000000000000100001010000011; //== lbu    x5      ,   x0(0)              ==//
    cpu.i_cache[0x0000001c] = 0b00000000000000000000000001101111; //== jal    x0      ,   0                  ==//
//=============================================================================================================//
}

void test2_fibonacci(CPU &cpu) {
//=============================================================================================================//
//==================================================================== intr   rd      ,   rs1     ,   rs2    ==//
//=============================================================================================================//
    cpu.i_cache[0x00000000] = 0b00000000110100000000001010010011; //== addi   x5      ,   x0      ,   0xD    ==//
    cpu.i_cache[0x00000004] = 0b00000000000000000000000010010011; //== addi   x1      ,   x0      ,   0x0    ==//
    cpu.i_cache[0x00000008] = 0b00000000000100000000000100010011; //== addi   x2      ,   x0      ,   0x1    ==//
    cpu.i_cache[0x0000000c] = 0b00000000000000000000000110010011; //== addi   x3      ,   x0      ,   0x0    ==//
    cpu.i_cache[0x00000010] = 0b00000000010100011000110001100011; //== beq    x3      ,   x5      ,   +24    ==//
    cpu.i_cache[0x00000014] = 0b00000000001000001000001000110011; //== add    x4      ,   x1      ,   x2     ==//
    cpu.i_cache[0x00000018] = 0b00000000000000010000000010110011; //== add    x1      ,   x2      ,   x0     ==//
    cpu.i_cache[0x0000001c] = 0b00000000000000100000000100110011; //== add    x2      ,   x4      ,   x0     ==//
    cpu.i_cache[0x00000020] = 0b00000000000100011000000110010011; //== addi   x3      ,   x3      ,   0x1    ==//
    cpu.i_cache[0x00000024] = 0b11111110110111111111000001101111; //== jal    x0      ,   -20                ==//
    cpu.i_cache[0x00000028] = 0b00000000001000000001000000100011; //== sh     x0(0)   ,   x2                 ==//
    cpu.i_cache[0x0000002c] = 0b00000000000000000000000001101111; //== jal    x0      ,   0                  ==//
//=============================================================================================================//
}

int main() {
    CPU cpu;
    
    printf("======================================= Test 1 =======================================\n");
    cpu.reset();
    test1_simple_test(cpu);
    for (int i = 0; i < 10; i++) {
        cpu.run_one_tick();
        printf("Cycle %03llu -> x1=0x%08x, x2=0x%08x, x3=0x%08x, x4=0x%08x, x5=0x%08x, Mem[0]=0x%08x\n",
            cpu.tick, cpu.rf[1], cpu.rf[2], cpu.rf[3], cpu.rf[4], cpu.rf[5], cpu.d_cache[0]);
    }

    printf("======================================= Test 2 =======================================\n");
    cpu.reset();
    test2_fibonacci(cpu);
    for (int i = 0; i < 90; i++) {
        cpu.run_one_tick();
        printf("Cycle %03llu -> x1=0x%08x, x2=0x%08x, x3=0x%08x, x4=0x%08x, x5=0x%08x, Mem[0]=0x%08x\n",
            cpu.tick, cpu.rf[1], cpu.rf[2], cpu.rf[3], cpu.rf[4], cpu.rf[5], cpu.d_cache[0]);
    }

    printf("====================================== Test End ======================================\n");
    
    return 0;
}
