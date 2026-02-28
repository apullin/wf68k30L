-- Explicit binding configuration for GHDL simulation.
-- GHDL's default component binding sometimes fails for component
-- instantiation; this configuration explicitly maps all components
-- to their work-library entities.

configuration WF68K30L_TOP_CFG of WF68K30L_TOP is
    for STRUCTURE
        for I_ADDRESSREGISTERS : WF68K30L_ADDRESS_REGISTERS
            use entity work.WF68K30L_ADDRESS_REGISTERS(BEHAVIOR);
        end for;
        for I_ALU : WF68K30L_ALU
            use entity work.WF68K30L_ALU(BEHAVIOUR);
        end for;
        for I_BUS_IF : WF68K30L_BUS_INTERFACE
            use entity work.WF68K30L_BUS_INTERFACE(BEHAVIOR);
        end for;
        for I_CONTROL : WF68K30L_CONTROL
            use entity work.WF68K30L_CONTROL(BEHAVIOUR);
        end for;
        for I_DATA_REGISTERS : WF68K30L_DATA_REGISTERS
            use entity work.WF68K30L_DATA_REGISTERS(BEHAVIOUR);
        end for;
        for I_EXC_HANDLER : WF68K30L_EXCEPTION_HANDLER
            use entity work.WF68K30L_EXCEPTION_HANDLER(BEHAVIOR);
        end for;
        for I_OPCODE_DECODER : WF68K30L_OPCODE_DECODER
            use entity work.WF68K30L_OPCODE_DECODER(BEHAVIOR);
        end for;
    end for;
end configuration WF68K30L_TOP_CFG;
