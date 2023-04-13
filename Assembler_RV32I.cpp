#include <cmath>
#include <array>
#include <cctype>
#include <vector>
#include <bitset>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <functional>
#include <unordered_map>

struct tokenized_code {
    std::string opcode;
    std::string rd;
    std::string rs1;
    std::string rs2;
    std::string imm;
};

// Map opcode names to their corresponding numbers and types
std::unordered_map<std::string, std::array<uint8_t, 4>> opcode_map =
{
    // R-Type Instructions
    {"add"  , {0b0110011, 0b000, 0b0000000, 'r'}},
    {"sub"  , {0b0110011, 0b000, 0b0100000, 'r'}},
    {"sll"  , {0b0110011, 0b001, 0b0000000, 'r'}},
    {"slt"  , {0b0110011, 0b010, 0b0000000, 'r'}},
    {"sltu" , {0b0110011, 0b011, 0b0000000, 'r'}},
    {"xor"  , {0b0110011, 0b100, 0b0000000, 'r'}},
    {"srl"  , {0b0110011, 0b101, 0b0000000, 'r'}},
    {"sra"  , {0b0110011, 0b101, 0b0100000, 'r'}},
    {"or"   , {0b0110011, 0b110, 0b0000000, 'r'}},
    {"and"  , {0b0110011, 0b111, 0b0000000, 'r'}},

    // I-Type Instructions
    {"jalr" , {0b1100111, 0b000, 0b0000000, 'i'}},
    {"lb"   , {0b0000011, 0b000, 0b0000000, 'i'}},
    {"lh"   , {0b0000011, 0b001, 0b0000000, 'i'}},
    {"lw"   , {0b0000011, 0b010, 0b0000000, 'i'}},
    {"lbu"  , {0b0000011, 0b100, 0b0000000, 'i'}},
    {"lhu"  , {0b0000011, 0b101, 0b0000000, 'i'}},
    {"addi" , {0b0010011, 0b000, 0b0000000, 'i'}},
    {"slli" , {0b0010011, 0b001, 0b0000000, 'i'}},
    {"slti" , {0b0010011, 0b010, 0b0000000, 'i'}},
    {"sltiu", {0b0010011, 0b011, 0b0000000, 'i'}},
    {"xori" , {0b0010011, 0b100, 0b0000000, 'i'}},
    {"srli" , {0b0010011, 0b101, 0b0000000, 'i'}},
    {"srai" , {0b0010011, 0b101, 0b0100000, 'i'}},
    {"ori"  , {0b0010011, 0b110, 0b0000000, 'i'}},
    {"andi" , {0b0010011, 0b111, 0b0000000, 'i'}},

    // S-Type Instructions
    {"sb"   , {0b0100011, 0b000, 0b0000000, 's'}},
    {"sh"   , {0b0100011, 0b001, 0b0000000, 's'}},
    {"sw"   , {0b0100011, 0b010, 0b0000000, 's'}},

    // B-Type Instructions
    {"beq"  , {0b1100011, 0b000, 0b0000000, 'b'}},
    {"bne"  , {0b1100011, 0b001, 0b0000000, 'b'}},
    {"blt"  , {0b1100011, 0b100, 0b0000000, 'b'}},
    {"bge"  , {0b1100011, 0b101, 0b0000000, 'b'}},
    {"bltu" , {0b1100011, 0b110, 0b0000000, 'b'}},
    {"bgeu" , {0b1100011, 0b111, 0b0000000, 'b'}},

    // U-Type Instructions
    {"lui"  , {0b0110111, 0b000, 0b0000000, 'u'}},
    {"auipc", {0b0010111, 0b000, 0b0000000, 'u'}},

    // J-Type Instruction
    {"jal"  , {0b1101111, 0b000, 0b0000000, 'j'}}
};

// Map register names to their corresponding numbers
std::unordered_map<std::string, uint8_t> reg_map = 
{
        {"x0" , 0 }, {"x1" , 1 }, {"x2",  2 }, {"x3" , 3 }, {"x4" , 4 }, {"x5" , 5 },
        {"x6" , 6 }, {"x7" , 7 }, {"x8",  8 }, {"x9" , 9 }, {"x10", 10}, {"x11", 11},
        {"x12", 12}, {"x13", 13}, {"x14", 14}, {"x15", 15}, {"x16", 16}, {"x17", 17},
        {"x18", 18}, {"x19", 19}, {"x20", 20}, {"x21", 21}, {"x22", 22}, {"x23", 23},
        {"x24", 24}, {"x25", 25}, {"x26", 26}, {"x27", 27}, {"x28", 28}, {"x29", 29},
        {"x30", 30}, {"x31", 31}
};

std::string replace(std::string src, std::string str_to_find, std::string str_to_replace) {
    size_t pos = 0;
    while ((pos = src.find(str_to_find, pos)) != std::string::npos) {
        src.replace(pos, str_to_find.length(), str_to_replace);
        pos += str_to_replace.length();
    }
    return src;
}

std::string trim(const std::string str) {
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    
    return (first < last) ? str.substr(first, last - first + 1) : "";
}

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c){ return std::tolower(c); });

    return s;
}

uint32_t convert_to_int(std::string num) {
    int base = 10; // default to decimal
    int start = (num[0] == '-') ? 1 : 0;
    int base_char_idx = start + 1;
    
    if (num.length() >= 2) {
        if (num[start] == '0') {
            if (num[base_char_idx] == 'b' || num[base_char_idx] == 'B') {
                base = 2;
                start += 2;
            } else if (num[base_char_idx] == 'o' || num[base_char_idx] == 'O') {
                base = 8;
                start += 2;
            } else if (num[base_char_idx] == 'd' || num[base_char_idx] == 'D') {
                base = 10;
                start += 2;
            } else if (num[base_char_idx] == 'x' || num[base_char_idx] == 'X') {
                base = 16;
                start += 2;
            }
        }
    }
    
    int32_t result = 0;
    for (int i = start; i < num.length(); i++) {
        char digit = num[i];
        int value = 0;
        if (digit >= '0' && digit <= '9') {
            value = digit - '0';
        } else if (digit >= 'A' && digit <= 'F') {
            value = digit - 'A' + 10;
        } else if (digit >= 'a' && digit <= 'f') {
            value = digit - 'a' + 10;
        }
        result += value * std::pow(base, num.length() - 1 - i);
    }

    return (num[0] == '-') ? result * -1 : result;
}

uint32_t encode_r_type(std::string opcode, std::string rd, std::string rs1, std::string rs2) {
    uint32_t opcode_num = opcode_map[opcode][0]; // opcode for R-type instruction
    uint32_t funct3 = opcode_map[opcode][1]; // funct3 for R-type instruction
    uint32_t funct7 = opcode_map[opcode][2]; // funct7 for R-type instruction
    uint32_t rd_num = reg_map[rd]; // get register number from rd
    uint32_t rs1_num = reg_map[rs1]; // get register number from rs1
    uint32_t rs2_num = reg_map[rs2]; // get register number from rs2

    // Encode the instruction
    return funct7 << 25 | rs2_num << 20 | rs1_num << 15 | funct3 << 12 | rd_num << 7 | opcode_num;
}

uint32_t encode_i_type(std::string opcode, std::string rd, std::string rs1, std::string imm) {
    uint32_t opcode_num = opcode_map[opcode][0]; // opcode for I-type instruction
    uint32_t funct3 = opcode_map[opcode][1]; // funct3 for I-type instruction
    uint32_t funct7 = opcode_map[opcode][2]; // funct3 for I-type instruction
    uint8_t rd_num = reg_map[rd]; // get register number from rd
    uint8_t rs1_num = reg_map[rs1]; // get register number from rs1
    // get immediate number from imm
    int32_t imm_num = (funct3 == 0b101 || funct3 == 0b001) ? funct7 << 5 | (convert_to_int(imm) & 0x1F) : convert_to_int(imm);

    // Encode the instruction
    return imm_num << 20 | rs1_num << 15 | funct3 << 12 | rd_num << 7 | opcode_num;
}

uint32_t encode_s_type(std::string opcode, std::string rs1, std::string rs2, std::string imm) {
    uint32_t opcode_num = opcode_map[opcode][0]; // opcode for S-type instruction
    uint32_t funct3 = opcode_map[opcode][1]; // funct3 for S-type instruction
    uint32_t rs1_num = reg_map[rs1]; // get register number from rs1
    uint32_t rs2_num = reg_map[rs2]; // get register number from rs2
    int32_t imm_num = convert_to_int(imm); // get immediate number from imm

    // Parse the immediate value and split it into its two parts
    uint32_t imm_11_5, imm_4_0;
    imm_11_5 = (imm_num >> 5) & 0x7f;
    imm_4_0 = imm_num & 0x1f;

    // Encode the instruction
    return (funct3 << 12) | (rs1_num << 15) | (rs2_num << 20) | (imm_11_5 << 25) | (imm_4_0 << 7) | opcode_num;
}

uint32_t encode_b_type(std::string opcode, std::string rs1, std::string rs2, std::string imm) {
    uint32_t opcode_num = opcode_map[opcode][0]; // opcode for B-type instruction
    uint32_t funct3 = opcode_map[opcode][1]; // funct3 for B-type instruction
    uint32_t rs1_num = reg_map[rs1]; // get register number from rs1
    uint32_t rs2_num = reg_map[rs2]; // get register number from rs2
    int32_t imm_num = convert_to_int(imm); // get immediate number from imm

    // Parse the immediate value and split it into its four parts
    uint32_t imm_12, imm_10_5, imm_4_1, imm_11;
    imm_12 = (imm_num >> 12) & 0x1;
    imm_11 = (imm_num >> 11) & 0x1;
    imm_10_5 = (imm_num >> 5) & 0x3f;
    imm_4_1 = (imm_num >> 1) & 0xf;

    // Encode the instruction
    return (funct3 << 12) | (rs1_num << 15) | (rs2_num << 20) | (imm_12 << 31) | (imm_11 << 7) | (imm_10_5 << 25) | (imm_4_1 << 8) | opcode_num;
}

uint32_t encode_u_type(std::string opcode, std::string rd, std::string imm) {
    uint32_t opcode_num = opcode_map[opcode][0];
    uint32_t rd_num = reg_map[rd]; // get register number from rd
    uint32_t imm_num = convert_to_int(imm); // get immediate number from imm

    // Encode the instruction
    return (imm_num << 12) | (rd_num << 7) | opcode_num;
}

uint32_t encode_j_type(std::string opcode, std::string rd, std::string imm) {
    uint32_t opcode_num = opcode_map[opcode][0];
    uint32_t rd_num = reg_map[rd]; // get register number from rd
    int32_t imm_num = convert_to_int(imm); // get immediate number from imm

    uint32_t imm_20, imm_10_1, imm_11, imm_19_12;
    imm_20 = (imm_num >> 20) & 0x1;
    imm_19_12 = (imm_num >> 12) & 0xff;
    imm_11 = (imm_num >> 11) & 0x1;
    imm_10_1 = (imm_num >> 1) & 0x3ff;

    // Encode the instruction
    return (imm_20 << 31) | (imm_19_12 << 12) | (imm_11 << 20) | (imm_10_1 << 21) | (rd_num << 7) | opcode_num;
}

tokenized_code tokenize(std::string input_assembly_line) {
    std::stringstream ss(input_assembly_line);
    std::string opcode, rd, rs1, rs2, imm;

    // Extract opcode
    std::getline(ss, opcode, ' ');

    // Check the opcode to determine instruction type
    switch (opcode_map[opcode][3])
    {
        case 'r': // R-Type instruction
            // Extract rd
            std::getline(ss, rd, ',');

            // Extract rs1
            std::getline(ss >> std::ws, rs1, ',');

            // Extract rs2
            std::getline(ss >> std::ws, rs2);

            break;
        case 'i': // I-Type instruction
            // Extract rd
            std::getline(ss, rd, ',');

            // If instruction was 'J'ALR, 'L'B, 'L'H, 'L'W, 'L'BU or 'L'HU extract it like this
            if (opcode[0] == 'j' || opcode[0] == 'l') {
                // Extract imm
                std::getline(ss >> std::ws, imm, '(');

                // Extract rs1
                std::getline(ss >> std::ws, rs1, ')');
            } else {
                // Extract rs1
                std::getline(ss >> std::ws, rs1, ',');

                // Extract imm
                std::getline(ss >> std::ws, imm);
            }

            break;
        case 's': // S-Type instruction
            // Extract rs2
            std::getline(ss, rs2, ',');

            // Extract imm
            std::getline(ss >> std::ws, imm, '(');

            // Extract rs1
            std::getline(ss >> std::ws, rs1, ')');

            break;
        case 'b': // B-Type instruction
            // Extract rs1
            std::getline(ss, rs1, ',');

            // Extract rs2
            std::getline(ss >> std::ws, rs2, ',');

            // Extract imm
            std::getline(ss >> std::ws, imm);

            break;
        case 'u': // U-Type instruction
            // Extract rd
            std::getline(ss, rd, ',');

            // Extract imm
            std::getline(ss >> std::ws, imm);

            break;
        case 'j': // J-Type instruction
            // Extract rd
            std::getline(ss, rd, ',');

            // Extract imm
            std::getline(ss >> std::ws, imm);
    }

    // Remove any whitespace from tokens
    opcode.erase(std::remove_if(opcode.begin(), opcode.end(), ::isspace), opcode.end());
    rd.erase(std::remove_if(rd.begin(), rd.end(), ::isspace), rd.end());
    rs1.erase(std::remove_if(rs1.begin(), rs1.end(), ::isspace), rs1.end());
    rs2.erase(std::remove_if(rs2.begin(), rs2.end(), ::isspace), rs2.end());
    imm.erase(std::remove_if(imm.begin(), imm.end(), ::isspace), imm.end());

    return {opcode, rd, rs1, rs2, imm};
}

uint32_t get_machine_code(std::string line) {
    tokenized_code tokens = tokenize(to_lower(line));

    switch (opcode_map[tokens.opcode][3]) {
        case 'r':
            return encode_r_type(tokens.opcode, tokens.rd, tokens.rs1, tokens.rs2);
        case 'i':
            return encode_i_type(tokens.opcode, tokens.rd, tokens.rs1, tokens.imm);
        case 's':
            return encode_s_type(tokens.opcode, tokens.rs1, tokens.rs2, tokens.imm);
        case 'b':
            return encode_b_type(tokens.opcode, tokens.rs1, tokens.rs2, tokens.imm);
        case 'u':
            return encode_u_type(tokens.opcode, tokens.rd, tokens.imm);
        case 'j':
            return encode_j_type(tokens.opcode, tokens.rd, tokens.imm);
        default:
            return 0;
    }
}

std::string assemble(std::string program) {
    std::string output = "";
    std::string line;
    std::unordered_map<uint32_t, std::string> lines;
    std::unordered_map<std::string, uint32_t> labels;

    program = (program[program.length() - 1] != '\n') ? program + '\n' : program;

    // Separate each line of code and stores it in `lines` and store address of any labeled lines in `labels`
    for (int i = 0, addr = 0; i < program.length(); i++) {
        if (program[i] == '\n') {
            if (line.empty()) continue;
            std::string line_trimmed = trim(line);
            if (line_trimmed.empty()) continue;
            
            if (line_trimmed.back() == ':') {
                std::string label_name = line_trimmed.substr(0, line_trimmed.length() - 1);
                labels[label_name] = addr;
            } else {
                lines[addr++] = line_trimmed;
            }
            line = "";
        } else {
            line += program[i];
        }
    }

    // Iterate through the lines and replace any labels with their corresponding addresses, and generate the machine code for each line.
    for (int i = 0; i < lines.size(); i++) {
        line = lines[i];
        for (auto& label : labels) {
            int32_t addr_to_replace = 4 * (label.second - i);
            std::string addr_to_str = std::to_string(addr_to_replace);
            line = replace(line, label.first, addr_to_str);
        }

        // Get machine code and convert it to bit string
        std::string machine_code = "0b" + std::bitset<32>(get_machine_code(line)).to_string();

        // Convert the integer to a hex string
        std::stringstream ss;
        ss << "0x" << std::hex << std::setw(8) << std::setfill('0') << (4 * i);
        std::string line_addr = ss.str();

        // Final output
        output += "[" + line_addr + "] = " + machine_code + '\n';
    }

    return output;
}

std::string test1_simple_test() {
    return
        "addi   x1      ,   x0      ,   0x1    \n" \
        "addi   x2      ,   x0      ,   0x2    \n" \
        "add    x2      ,   x1      ,   x2     \n" \
        "addi   x3      ,   x0      ,   0x0EFE \n" \
        "sb     x3      ,   0(x0)              \n" \
        "lb     x4      ,   0(x0)              \n" \
        "lbu    x5      ,   0(x0)              \n" \
        "jal    x0      ,   0                  \n";
}

std::string test2_fibonacci() {
    return 
        "addi   x5      ,   x0      ,   0xD    \n" \
        "addi   x1      ,   x0      ,   0x0    \n" \
        "addi   x2      ,   x0      ,   0x1    \n" \
        "addi   x3      ,   x0      ,   0x0    \n" \
        "loop1:                                \n" \
        "beq    x3      ,   x5      ,   loop2  \n" \
        "add    x4      ,   x1      ,   x2     \n" \
        "add    x1      ,   x2      ,   x0     \n" \
        "add    x2      ,   x4      ,   x0     \n" \
        "addi   x3      ,   x3      ,   0x1    \n" \
        "jal    x0      ,   loop1              \n" \
        "loop2:                                \n" \
        "sh     x2      ,   0(x0)              \n" \
        "halt:                                 \n" \
        "jal    x0      ,   halt               \n";
}

int main() {
    std::cout << "Machine codes for Test1: \n\n" << assemble(test1_simple_test()) << std::endl;
    std::cout << "Machine codes for Test2: \n\n" << assemble(test2_fibonacci()) << std::endl;

    return 0;
}