function [Hdb, Fdbt, Cdb, Adc] = MPC_simplification(Ad, Bd, Cd, Dd, hz)
    % db - double ber
    % dbt -double ber transpose
    % dc - double ber circumflex
    
    A_aug = [Ad, Bd; zeros(length(Bd(1, :)), length(Ad(1, :))), eye(length(Bd(1, :)))];
    B_aug = [Bd; eye(length(Bd(1, :)))];
    C_aug = [Cd, zeros(length(Cd(:, 1)), length(Bd(1, :)))];
    D_aug = Dd;
    
    constants = initial_constants();
    Q = constants{8};
    S = constants{9};
    R = constants{10};
    
    CQC = C_aug' * Q * C_aug;
    CSC = C_aug' * S * C_aug;
    QC  = Q * C_aug;
    SC  = S * C_aug;
    
    Qdb = zeros(length(CQC(:, 1)) * hz, length(CQC(1, :)) * hz);
    Tdb = zeros(length(QC(:, 1)) * hz, length(QC(1, :)) * hz);
    Rdb = zeros(length(R(:, 1)) * hz, length(R(1, :)) * hz);
    Cdb = zeros(length(B_aug(:, 1)) * hz, length(B_aug(1, :)) * hz);
    Adc = zeros(length(A_aug(:, 1)) * hz, length(A_aug(1, :)));
    
    for i = 1:hz
        if i == hz
            Qdb(1 + length(CSC(:, 1)) * (i - 1) : length(CSC(:, 1)) * i, 1 + length(CSC(1, :)) * (i - 1) : length(CSC(1, :)) * i) = CSC;
            Tdb(1 + length(SC(:, 1)) * (i - 1) : length(SC(:, 1)) * i, 1 + length(SC(1, :)) * (i - 1) : length(SC(1, :)) * i) = SC;
        else
            Qdb(1 + length(CQC(:, 1)) * (i - 1) : length(CQC(:, 1)) * i, 1 + length(CQC(1, :)) * (i - 1) : length(CQC(1, :)) * i) = CQC;
            Tdb(1 + length(QC(:, 1)) * (i - 1) : length(QC(:, 1)) * i, 1 + length(QC(1, :)) * (i - 1) : length(QC(1, :)) * i) = QC;
        end
        
        Rdb(1 + length(R(:, 1)) * (i - 1) : length(R(:, 1)) * i, 1 + length(R(1, :)) * (i - 1) : length(R(1, :)) * i) = R;
        
        for j = 1:hz
            if j <= i
                Cdb(1 + length(B_aug(:, 1)) * (i - 1) : length(B_aug(:, 1)) * i, 1 + length(B_aug(1, :)) * (j - 1) : length(B_aug(1, :)) * j) = A_aug^(i - j) * B_aug;
            end
        end
        
        Adc(1 + length(A_aug(:, 1)) * (i - 1) : length(A_aug(:, 1)) * i, 1 : length(A_aug(1, :))) = A_aug^(i);
    end
    
    Hdb  = Cdb' * Qdb * Cdb + Rdb;
    Fdbt = [Adc' * Qdb * Cdb; -Tdb * Cdb];
end