%% dataVisualization.m

%% Read from Serial Monitor 
clear
clear all

port = "COM5"; 
baudRate = 115200; 

serialPort = serialport(port,baudRate,Timeout = 600); 

data = []; 

row = 1; 

while(true)
    serialPortData = readline(serialPort);
    % disp("Received: " + serialPortData);
    values = str2double(split(serialPortData, ','));

    if numel(values) == 8 && all(~isnan(values))
        data(row, :) = values';
        row = row + 1; 
    else
        warning("Line %d skipped due to unexpected format.", i);
    end
end 

clear serialPort;