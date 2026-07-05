def r_type(funct7, rs2, rs1, funct3, rd, opcode):
    return (funct7<<25)|(rs2<<20)|(rs1<<15)|(funct3<<12)|(rd<<7)|opcode

def i_type(imm, rs1, funct3, rd, opcode):
    imm = imm & 0xFFF
    return (imm<<20)|(rs1<<15)|(funct3<<12)|(rd<<7)|opcode

def s_type(imm, rs2, rs1, funct3, opcode):
    imm = imm & 0xFFF
    imm11_5 = (imm>>5)&0x7F
    imm4_0  = imm & 0x1F
    return (imm11_5<<25)|(rs2<<20)|(rs1<<15)|(funct3<<12)|(imm4_0<<7)|opcode

def b_type(imm, rs2, rs1, funct3, opcode):
    # imm is byte offset, must be even
    imm = imm & 0x1FFF
    b12   = (imm>>12)&1
    b10_5 = (imm>>5)&0x3F
    b4_1  = (imm>>1)&0xF
    b11   = (imm>>11)&1
    return (b12<<31)|(b10_5<<25)|(rs2<<20)|(rs1<<15)|(funct3<<12)|(b4_1<<8)|(b11<<7)|opcode

def u_type(imm, rd, opcode):
    return ((imm & 0xFFFFF)<<12)|(rd<<7)|opcode

def j_type(imm, rd, opcode):
    imm = imm & 0x1FFFFF
    b20    = (imm>>20)&1
    b10_1  = (imm>>1)&0x3FF
    b11    = (imm>>11)&1
    b19_12 = (imm>>12)&0xFF
    return (b20<<31)|(b19_12<<12)|(b11<<20)|(b10_1<<21)|(rd<<7)|opcode

OP_RTYPE=0b0110011
OP_ITYPE=0b0010011
OP_LOAD =0b0000011
OP_STORE=0b0100011
OP_BRANCH=0b1100011
OP_JAL  =0b1101111
OP_JALR =0b1100111
OP_LUI  =0b0110111
OP_AUIPC=0b0010111

def ADDI(rd,rs1,imm): return i_type(imm,rs1,0b000,rd,OP_ITYPE)
def ADD(rd,rs1,rs2):  return r_type(0,rs2,rs1,0b000,rd,OP_RTYPE)
def SUB(rd,rs1,rs2):  return r_type(0b0100000,rs2,rs1,0b000,rd,OP_RTYPE)
def AND(rd,rs1,rs2):  return r_type(0,rs2,rs1,0b111,rd,OP_RTYPE)
def OR(rd,rs1,rs2):   return r_type(0,rs2,rs1,0b110,rd,OP_RTYPE)
def XOR(rd,rs1,rs2):  return r_type(0,rs2,rs1,0b100,rd,OP_RTYPE)
def SW(rs2,imm,rs1):  return s_type(imm,rs2,rs1,0b010,OP_STORE)
def LW(rd,imm,rs1):   return i_type(imm,rs1,0b010,rd,OP_LOAD)
def BEQ(rs1,rs2,imm): return b_type(imm,rs2,rs1,0b000,OP_BRANCH)
def JAL(rd,imm):      return j_type(imm,rd,OP_JAL)
def LUI(rd,imm20):    return u_type(imm20,rd,OP_LUI)

prog = []
prog.append(ADDI(1,0,5))          # 0x00 addi x1,x0,5
prog.append(ADDI(2,0,10))         # 0x04 addi x2,x0,10
prog.append(ADD(3,1,2))           # 0x08 add x3,x1,x2   -> 15
prog.append(SUB(4,2,1))           # 0x0C sub x4,x2,x1   -> 5
prog.append(AND(5,1,2))           # 0x10 and x5,x1,x2   -> 0
prog.append(OR(6,1,2))            # 0x14 or  x6,x1,x2   -> 15
prog.append(XOR(7,1,2))           # 0x18 xor x7,x1,x2   -> 15
prog.append(SW(3,0,0))            # 0x1C sw x3,0(x0)
prog.append(LW(8,0,0))            # 0x20 lw x8,0(x0)    -> 15
prog.append(BEQ(3,8,8))           # 0x24 beq x3,x8,+8 -> to 0x2C
prog.append(ADDI(9,0,999))        # 0x28 (skipped)
prog.append(JAL(10,8))            # 0x2C jal x10,+8 -> to 0x34, x10=0x30
prog.append(ADDI(11,0,888))       # 0x30 (skipped)
prog.append(LUI(12,4))            # 0x34 lui x12,4      -> x12=0x4000
prog.append(SW(3,0,12))           # 0x38 sw x3,0(x12)   -> AXI write, wdata=15
prog.append(LW(13,0,12))          # 0x3C lw x13,0(x12)  -> AXI read
prog.append(JAL(0,0))             # 0x40 jal x0,0       -> infinite self loop

with open('imem.hex','w') as f:
    for instr in prog:
        f.write('%08x\n' % (instr & 0xFFFFFFFF))

for i,instr in enumerate(prog):
    print('%08x: %08x' % (i*4, instr))
