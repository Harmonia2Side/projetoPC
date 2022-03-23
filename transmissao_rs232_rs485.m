% =========================================================================
% 
% C�digo para simula��o de uma transmiss�o de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
% 
% =========================================================================

%% Inicializa��o 
clc, close all, clear all


%% .................  Parametriza��o do Sistema  ...........................
M = 2; % N�vel de Modula��o (Mapeamento para codifica��o digital)
nsamp = 4; % Fator de reamostragem
snr = 12; % Rela��o sinal ru�do (dado em 3db / Dobro da potencia)
nData = 10e3;


%% .................  Transmiss�o  ......................................... 

%Gera��o de dados aleat�rios
data_in = randi([0 M-1], nData, 1);

% % Convers�o de bits em pulsos
% x_pulse = pammod(dataIn, M); 

x_pulse = data_in

% Conforma��o de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular


%% -----------------  Canal  -----------------------------------------------
% Adicionando Ru�do
x_up_noise = awgn(x_up,snr,'measured');

% -------------------------------------------------------------------------

%% ................. Recep��o ..............................................

% Filtragem do ru�do AWGN do sinal
x_up_filtrado = x_up_noise;
len = length(x_up_noise);
t_hold = 0; % Defini��o do threshold (limiar) do sinal 

% Codifica��o NRZ Polar
for i = 1:len
    if (x_up_noise(i)>t_hold)
       x_up_filtrado(i) = 1;
    else 
        x_up_filtrado(i) = -1;
    end
end

% Deconforma��o de pulso
x_dp = intdump(x_up_filtrado, nsamp);

% % Convers�o de pulsos em bits
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

% Exibi��o do pulso no canal de transmissao
figure;
plot(x_up,'LineWidth',0.5);
hold all;
plot(x_up_noise),xlim([0 200]), ylim([-2 2]), title('Canal - (Pulso + AWGN)');

% Exibi��o dos pulsos no sistema de comunica��o
figure;
subplot(3,1,1), plot(x_up), xlim([0 200]), ylim([-1.5 1.5]), title('Transmiss�o');
subplot(3,1,2), hold on, plot(x_up),plot(x_up_noise), xlim([0 200]), ylim([-1.5 1.5]), title('Canal - (Pulso + AWGN)');
subplot(3,1,3), plot(x_up_filtrado,'Color','green'), xlim([0 200]), ylim([-1.5 1.5]), title('Recep��o');


% Plotagem do Diagrama de Olho da sa�da do canal
eyediagram(x_up_noise(1:end/10000), 3)




