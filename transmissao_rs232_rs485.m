% =========================================================================
% 
% Código para simulação de uma transmissão de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
% 
% =========================================================================

%% Inicialização 
clc, close all, clear all


%% .................  Parametrização do Sistema  ...........................
M = 2; % Nível de Modulação (Mapeamento para codificação digital)
nsamp = 4; % Fator de reamostragem
snr = 12; % Relação sinal ruído (dado em 3db / Dobro da potencia)
nData = 10e3;


%% .................  Transmissão  ......................................... 

%Geração de dados aleatórios
data_in = randi([0 M-1], nData, 1);

% % Conversão de bits em pulsos
% x_pulse = pammod(dataIn, M); 

x_pulse = data_in

% Conformação de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular


%% -----------------  Canal  -----------------------------------------------
% Adicionando Ruído
x_up_noise = awgn(x_up,snr,'measured');

% -------------------------------------------------------------------------

%% ................. Recepção ..............................................

% Filtragem do ruído AWGN do sinal
x_up_filtrado = x_up_noise;
len = length(x_up_noise);
t_hold = 0; % Definição do threshold (limiar) do sinal 

% Codificação NRZ Polar
for i = 1:len
    if (x_up_noise(i)>t_hold)
       x_up_filtrado(i) = 1;
    else 
        x_up_filtrado(i) = -1;
    end
end

% Deconformação de pulso
x_dp = intdump(x_up_filtrado, nsamp);

% % Conversão de pulsos em bits
% img_Rx_bin = pamdemod(x_dp, M);

data_out

% Calculo do BER do dado transmitido
bits_errados = 0;
i = 1;
while i <= length(img_Tx_bin)
    if (img_Tx_bin(i) ~= img_Rx_bin(i))
        bits_errados = bits_errados + 1;
    end
    i = i + 1;
end
format shortEng
BER = (bits_errados / length(img_Tx_bin)) % Calculo de BER




%% +++++++++++++++++++  Plotagens  +++++++++++++++++++++++++++++++++++++++++

% Exibição do pulso no canal de transmissao
figure;
plot(x_up,'LineWidth',0.5);
hold all;
plot(x_up_noise),xlim([0 200]), ylim([-2 2]), title('Canal - (Pulso + AWGN)');

% Exibição dos pulsos no sistema de comunicação
figure;
subplot(3,1,1), plot(x_up), xlim([0 200]), ylim([-1.5 1.5]), title('Transmissão');
subplot(3,1,2), hold on, plot(x_up),plot(x_up_noise), xlim([0 200]), ylim([-1.5 1.5]), title('Canal - (Pulso + AWGN)');
subplot(3,1,3), plot(x_up_filtrado,'Color','green'), xlim([0 200]), ylim([-1.5 1.5]), title('Recepção');


% Plotagem do Diagrama de Olho da saída do canal
eyediagram(x_up_noise(1:end/10000), 3)




