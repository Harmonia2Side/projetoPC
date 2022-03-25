% =========================================================================
%
% C�digo para simula��o de uma transmiss�o de um conjunto de dados MODBUS
% por meio dos protocolos RS485 e RS232.
%
% =========================================================================

%% Inicializa��o
clc, close all, clear all


%% .................  Parametriza��o do Sistema  ...........................
Rb = 10e1;          % Taxa de transmiss�o do sistema 10 x 10^6 = 10^7 = 10 Mbits/s
Tb = 1/Rb;          % Dura��o de um bit
M = 2;              % N�vel de Modulu��o (Mapeamento para codifica��o digital)
m = log2(M);        % bits / s�mbolo
Rs = Rb/m;          % Taxa de s�mbolos (sps - taxa s�mbolos por segundo ou baud rate)
Ts = 1/Rs;          % Dura��o no tempo de cada s�mbolo
nsamp = 4;          % Fator de reamostragem
% snr = 6;            % Rela��o sinal ru�do (dado em 3db / Dobro da potencia)
nData = 16*1024;    % N�mero de bits a serem transmitidos = 16kb

snr = 2; % Valor inicial da rela��o sinal ru�do(dado em 3db / Dobro da potencia)
snrMax = 20;

% .................  C�lculo de vari�veis iniciais  ......................
t = linspace(0,Ts*((nData-1)*nsamp), nData*nsamp).';


%% .................  Transmiss�o  .........................................

%Gera��o de dados aleat�rios
data_in = randi([0 M-1], nData, 1);

x_pulse = data_in; % Sinal bruto, sem modula��o
x_pulse_232 = data_in; % Sinal bruto, sem modula��o

% Conforma��o de pulso (Pulse Shaping) - Super Amostragem (Upsampling)
x_up = rectpulse(x_pulse, nsamp); % Filtro Retangular
x_up_232 = rectpulse(x_pulse_232, nsamp); % Filtro Retangular

% Codifica��o diferencial RS-485
x_b = double( x_up);
x_a = double(~x_up);

% Codifica��o single RS-232
x_232 = double(~x_up_232);

for i = 1:length(x_232)
    if (x_232(i) == 0)
        x_232(i) = -10;
    end
    if (x_232(i) == 1)
        x_232(i) = 10;
    end
end


%% Gera��o da curva BER x SNR
for j = 1:(snrMax-snr)

    %% -----------------  Canal  -----------------------------------------------
    % Adicionando Ru�do

    % Objeto aleat�rio
    S = RandStream('mt19937ar','Seed',5489);

    x_b_noise = awgn(x_b,snr,'measured', S);
    reset(S)
    x_a_noise = awgn(x_a,snr,'measured', S);
    reset(S)
    x_232_noise = awgn(x_232,snr,'measured', S);

    % -------------------------------------------------------------------------

    %% ................. Recep��o ..............................................

    len = length(x_a_noise);

    % Decodifica��o diferenial RS-485
    for i = 1:len
        if (x_a_noise(i) < x_b_noise(i))
            x_up_filtrado(i) = 1;
        else
            x_up_filtrado(i) = 0;
        end
    end


    % % Decodifica��o RS-232
    % for i = 1:len
    %     if (x_232_noise(i) > 3)
    %         x_up_filtrado_232(i) = 0;
    %     elseif (x_232_noise(i) < - 3)
    %             x_up_filtrado_232(i) = 1;
    %     else
    %         x_up_filtrado_232(i) = NaN;
    %     end
    % end

    % Decodifica��o RS-232 (ideal com threshold em zero)
    for i = 1:len
        if (x_232_noise(i) > 0)
            x_up_filtrado_232(i) = 0;
        end
        if (x_232_noise(i) < 0)
            x_up_filtrado_232(i) = 1;
        end
    end


    % Deconforma��o de pulso
    x_dp = intdump(x_up_filtrado, nsamp);
    x_dp_232 = intdump(x_up_filtrado_232, nsamp);


    data_out_485 = x_dp;
    % substituindo NaN por um n�mero qualquer que seja lido como erro na fun��o biterr
    % TF = isnan(x_dp_232);
    % x_dp_232(TF) = 0;
    data_out_232 = round(x_dp_232);


    % [nErrors_485, BER_485] = biterr(data_in, data_out_485');
    [nErrors_485, berData485(j)] = biterr(data_in, data_out_485');
    [nErrors_232, berData232(j)] = biterr(data_in, data_out_232');
    snrData485(j) = snr;
    snrData232(j) = snr;
    snr = snr + 1;
end


%% +++++++++++++++++++  Plotagens  +++++++++++++++++++++++++++++++++++++++++

% Limites do gr�fico
xMax = 0.2e-4;

% % Exibi��o do canal de transmiss�o A
% figure(1);
% plot(t, x_a,'LineWidth',0.5);
% hold all;
% plot(t, x_a_noise);
% xlim([0 xMax])
% ylim([-1.5 2])
% title('Fig1: Canal A (RS485)');
% saveas(gcf, 'Fig1: Canal A (RS485).png')
% 
% % Exibi��o do canal de transmiss�o B
% figure(2);
% plot(t, x_b,'LineWidth',0.5);
% hold all;
% plot(t, x_b_noise)
% xlim([0 xMax])
% ylim([-1.5 2])
% title('Fig2: Canal B (RS485)');
% saveas(gcf, 'Fig2: Canal B (RS485).png')
% 
% % Exibi��o do canal de transmiss�o RS232
% figure(3);
% plot(t, x_232,'LineWidth',0.5);
% hold all;
% plot(t, x_232_noise)
% xlim([0 xMax])
% % ylim([-1.5 2])
% title('Fig3: Canal (RS232)');
% saveas(gcf, 'Fig3: Canal (RS232).png')
% 
% % Exibi��o dos sinais transmitido e recebido
% figure(4);
% plot(t, x_up,'LineWidth',0.5);
% hold all;
% plot(t, x_up_filtrado);
% xlim([0 xMax])
% ylim([-1.5 2])
% title('Fig4: Sinal Transmitido e recebido (RS485)');
% legend('Sinal Transmitido','Sinal Recebido');
% saveas(gcf, 'Fig4: Sinal Transmitido e recebido (RS485).png')
% 
% % Exibi��o dos sinais RS232
% figure(5);
% yyaxis left
% plot(t, x_up_232,'LineWidth',0.5);
% xlim([0 xMax]);
% ylim([-2 1.5]);
% % title('Sinal Transmitido (RS232)');
% 
% hold all;
% 
% yyaxis right
% plot(t, x_up_filtrado_232,'LineWidth',0.5);
% xlim([0 xMax])
% ylim([-0.5 3])
% title('Fig5: Sinal Transmitido e recebido (RS232)');
% legend('Sinal Transmitido','Sinal Recebido');
% saveas(gcf, 'Fig5: Sinal Transmitido recebido (RS232).png');


% BER x SNR em logaritmo
figure;
semilogy(snrData232,berData232); 
% hold on
% semilogy(snrData485,berData485); 
xlabel('SNR [dbm]'); 
ylabel('BER'); 
title('Fig6: BER x SNR (SR232)')
xlim([2 snrMax]);
ylim([1e-5 1e-1])
saveas(gcf, 'Fig6: BER x SNR (SR232).png');


%% Compara��o com modula��o PAM com M = 2, 4, 8
[snrData_PAM,berData_PAM] = ber_snr_v2();
%Concatenando matrizes
snrData_plot = [snrData232' snrData_PAM];
berData_plot = [berData232' berData_PAM];


figure;
semilogy(snrData_plot,berData_plot); 
% semilogy(snrData232,berData232); 
legend('SR232','M = 2', 'M = 4', 'M = 8')
% hold on
% semilogy(snrData485,berData485); 
xlabel('SNR [dbm]'); 
ylabel('BER'); 
title('Fig7: Compara��o entre SR232 e PAM (M = 2,4,8)');
xlim([2 snrMax]);
% ylim([1e-5 1e-1])
% hold on
% [snrData_PAM,berData_PAM] = ber_snr_v2();
% semilogy(snrData_PAM,berData_PAM); xlabel('SNR [dbm]'); ylabel('BER'); 
% legend('M = 2', 'M = 4', 'M = 8');

