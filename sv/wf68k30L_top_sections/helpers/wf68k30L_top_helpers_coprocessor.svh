// Coprocessor interface classification helpers (HW-003 phase 4 model scope).

function automatic logic cpif_is_cpsave_ea_valid(
    input logic [2:0] mode,
    input logic [2:0] reg_fld
);
begin
    if (mode == 3'b111)
        cpif_is_cpsave_ea_valid = reg_fld < 3'b010; // (xxx).W/(xxx).L
    else
        cpif_is_cpsave_ea_valid = mode == 3'b010 || mode == 3'b100 || mode == 3'b101 || mode == 3'b110;
end
endfunction

function automatic logic cpif_is_cprestore_ea_valid(
    input logic [2:0] mode,
    input logic [2:0] reg_fld
);
begin
    if (mode == 3'b111)
        cpif_is_cprestore_ea_valid = reg_fld < 3'b100; // abs/PC-relative
    else
        cpif_is_cprestore_ea_valid = mode == 3'b010 || mode == 3'b011 || mode == 3'b101 || mode == 3'b110;
end
endfunction

// Non-MMU coprocessor opword classes that can initiate coprocessor protocol.
function automatic logic cpif_is_protocol_candidate(input logic [15:0] iw);
    logic [2:0] cp_type;
    logic [2:0] mode;
    logic [2:0] reg_fld;
begin
    cp_type = iw[8:6];
    mode = iw[5:3];
    reg_fld = iw[2:0];

    if (iw[15:12] != 4'hF || iw[11:9] == 3'b000)
        cpif_is_protocol_candidate = 1'b0;
    else begin
        case (cp_type)
            3'b000, 3'b001, 3'b010: cpif_is_protocol_candidate = 1'b1; // cpGEN / conditional / cpBcc
            3'b100: cpif_is_protocol_candidate = cpif_is_cpsave_ea_valid(mode, reg_fld);
            3'b101: cpif_is_protocol_candidate = cpif_is_cprestore_ea_valid(mode, reg_fld);
            default: cpif_is_protocol_candidate = 1'b0;
        endcase
    end
end
endfunction

// Coprocessor instruction category codes for model-state tracking.
function automatic logic [2:0] cpif_category(input logic [15:0] iw);
begin
    case (iw[8:6])
        3'b000: cpif_category = 3'd1; // cpGEN
        3'b001: cpif_category = 3'd2; // cpScc/cpDBcc/cpTRAPcc
        3'b010: cpif_category = 3'd3; // cpBcc
        3'b100: cpif_category = 3'd4; // cpSAVE
        3'b101: cpif_category = 3'd5; // cpRESTORE
        default: cpif_category = 3'd0; // none / unsupported
    endcase
end
endfunction

// CIR access used to initiate the instruction in this model.
function automatic logic [4:0] cpif_initiator_cir(input logic [2:0] cat);
begin
    case (cat)
        3'd1: cpif_initiator_cir = 5'h0A; // command CIR (cpGEN)
        3'd2: cpif_initiator_cir = 5'h0E; // condition CIR (conditional category)
        3'd3: cpif_initiator_cir = 5'h00; // response CIR read (cpBcc path model)
        3'd4: cpif_initiator_cir = 5'h04; // save CIR read
        3'd5: cpif_initiator_cir = 5'h06; // restore CIR write
        default: cpif_initiator_cir = 5'h00;
    endcase
end
endfunction

function automatic logic cpif_initiator_is_read(input logic [2:0] cat);
begin
    case (cat)
        3'd3, 3'd4: cpif_initiator_is_read = 1'b1; // cpBcc model / cpSAVE
        default: cpif_initiator_is_read = 1'b0;
    endcase
end
endfunction

// Number of modeled CIR protocol steps before a timeout (no response).
function automatic logic [3:0] cpif_protocol_steps(input logic [2:0] cat);
begin
    case (cat)
        3'd1: cpif_protocol_steps = 4'd4; // cpGEN
        3'd2: cpif_protocol_steps = 4'd3; // conditional
        3'd3: cpif_protocol_steps = 4'd2; // cpBcc
        3'd4: cpif_protocol_steps = 4'd2; // cpSAVE
        3'd5: cpif_protocol_steps = 4'd2; // cpRESTORE
        default: cpif_protocol_steps = 4'd1;
    endcase
end
endfunction

// Per-step modeled CIR bus sequence for each category.
function automatic logic [4:0] cpif_step_cir(
    input logic [2:0] cat,
    input logic [3:0] step_idx
);
begin
    case (cat)
        3'd1: begin // cpGEN
            case (step_idx)
                4'd0: cpif_step_cir = 5'h0A;
                4'd1: cpif_step_cir = 5'h00;
                4'd2: cpif_step_cir = 5'h0B;
                default: cpif_step_cir = 5'h00;
            endcase
        end
        3'd2: begin // conditional
            case (step_idx)
                4'd0: cpif_step_cir = 5'h0E;
                default: cpif_step_cir = 5'h00;
            endcase
        end
        3'd3: cpif_step_cir = 5'h00; // cpBcc
        3'd4: begin // cpSAVE
            case (step_idx)
                4'd0: cpif_step_cir = 5'h04;
                default: cpif_step_cir = 5'h00;
            endcase
        end
        3'd5: begin // cpRESTORE
            case (step_idx)
                4'd0: cpif_step_cir = 5'h06;
                default: cpif_step_cir = 5'h0A;
            endcase
        end
        default: cpif_step_cir = 5'h00;
    endcase
end
endfunction

function automatic logic cpif_step_is_read(
    input logic [2:0] cat,
    input logic [3:0] step_idx
);
begin
    case (cat)
        3'd1: cpif_step_is_read = (step_idx == 4'd1 || step_idx == 4'd3);
        3'd2: cpif_step_is_read = 1'b1;
        3'd3: cpif_step_is_read = 1'b1;
        3'd4: cpif_step_is_read = 1'b1;
        default: cpif_step_is_read = 1'b0;
    endcase
end
endfunction

function automatic logic [4:0] cpif_response_cir(input logic [2:0] cat);
begin
    case (cat)
        3'd1, 3'd2, 3'd3, 3'd4, 3'd5: cpif_response_cir = 5'h00;
        default: cpif_response_cir = 5'h00;
    endcase
end
endfunction
