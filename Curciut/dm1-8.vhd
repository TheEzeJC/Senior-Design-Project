library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity demux_1to8 is
    Port (
        data_in : in  std_logic;         -- Input data
        sel     : in  std_logic_vector(2 downto 0);  -- 3-bit select line
        data_out : out std_logic_vector(7 downto 0)  -- 8 output lines
    );
end demux_1to8;

architecture Behavioral of demux_1to8 is
begin
    process(data_in, sel)
    begin
        -- Initialize all outputs to '0'
        data_out <= (others => '0');

        -- Select the corresponding output based on 'sel'
        case sel is
            when "000" => data_out(0) <= data_in;
            when "001" => data_out(1) <= data_in;
            when "010" => data_out(2) <= data_in;
            when "011" => data_out(3) <= data_in;
            when "100" => data_out(4) <= data_in;
            when "101" => data_out(5) <= data_in;
            when "110" => data_out(6) <= data_in;
            when "111" => data_out(7) <= data_in;
            when others => data_out <= (others => '0');  -- Default case
        end case;
    end process;
end Behavioral;