% =========================================================================
% 
% Código para simulação de uma transmissão de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
% 
% =========================================================================

%% Inicialização 
clc, close all, clear all


%% .................  Parametrização do Sistema  ...........................
Rb = 10e6;          % Taxa de transmissão do sistema 10 x 10^6 = 10^7 = 10 Mbits/s
Tb = 1/Rb;          % Duração de um bit
M = 2;              % Nível de Modulução (Mapeamento para codificação digital)
m = log2(M);        % bits / símbolo
Rs = Rb/m;          % Taxa de símbolos (sps - taxa símbolos por segundo ou baud rate)
Ts = 1/Rs;          % Duração no tempo de cada símbolo
nsamp = 4;          % Fator de reamostragem
snr = 6;            % Relação sinal ruído (dado em 3db / Dobro da potencia)
nData = 16*1024;    % Número de bits a serem transmitidos = 16kb

% .................  Cálculo de variáveis iniciais  ......................
t = linspace(0,Ts*((nData-1)*nsamp), nData*nsamp).';


%% .................  Transmissão  ......................................... 

%Geração de dados aleatórios
data_in = randi([0 M-1], nData, 1);

x_pulse = data_in; % Sinal bruto, sem modulação

% Conformação de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular

    % Codificação diferencial RS-485
x_b = double( x_up);
x_a = double(~x_up);

%% -----------------  Canal  -----------------------------------------------
% Adicionando Ruído

% Objeto aleatório
S = RandStream('mt19937ar','Seed',5489);

x_b_noise = awgn(x_b,snr,'measured', S);
reset(S)
x_a_noise = awgn(x_a,snr,'measured', S);

% -------------------------------------------------------------------------

%% ................. Recepção ..............................................

len = length(x_a_noise);

% Filtro linear
% threshold

% Decodificação diferenial
for i = 1:len
    if (x_a_noise(i) < x_b_noise(i))
       x_up_filtrado(i) = 1;
    else 
        x_up_filtrado(i) = 0;
    end
end


% Deconformação de pulso
x_dp = intdump(x_up_filtrado, nsamp);


data_out = x_dp; % Sinal bruto, sem modulação

% Calculo do BER do dado transmitido
bits_errados = 0;
i = 1;
while i <= length(data_in)
    if (data_in(i) ~= data_out(i))
        bits_errados = bits_errados + 1;
    end
    i = i + 1;
end
format shortEng
BER = (bits_errados / length(data_in)); % Calculo de BER




%% +++++++++++++++++++  Plotagens  +++++++++++++++++++++++++++++++++++++++++

% Limites do gráfico
xMax = 0.5e-4

% Exibição do canal de transmissão A
figure;
plot(t, x_a,'LineWidth',0.5);
hold all;
plot(t, x_a_noise);
xlim([0 xMax])
ylim([-1.5 2]) 
title('Canal A');

% Exibição do canal de transmissão B
figure;
plot(t, x_b,'LineWidth',0.5);
hold all;
plot(t, x_b_noise)
xlim([0 xMax])
ylim([-1.5 2])
title('Canal B');

% Exibição dos sinais transmitido e recebido
figure;
plot(t, x_up,'LineWidth',0.5);
hold all;
plot(t, x_up_filtrado)
xlim([0 xMax])
ylim([-1.5 2])
title('Sinal Transmitido e recebido');
legend('Sinal Transmitido','Sinal Recebido')
