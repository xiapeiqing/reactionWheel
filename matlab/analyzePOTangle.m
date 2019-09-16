function analyzePOTangle()
%%
close all;
addpath D:\code\RadioLoc\matlab;
% set FlightBatterySettings to SensorCalibrationFactor=1 and SensorCalibrationOffset=0

fclose all;
ExpectedReadingLeft2Right = [-18 18];
ADCreading = [2754 1721];

ExpectedReadingLeft2Right = ExpectedReadingLeft2Right(:);
ADCreading = ADCreading(:);
Coeff = [];
for ii = 1:length(ADCreading)
    Coeff = [Coeff; ADCreading(ii) 1];
end
Est = (Coeff'*Coeff)^-1*Coeff'*ExpectedReadingLeft2Right;
reconstructed = ADCreading*Est(1)+Est(2);
fprintf(1,'\n-------------\nSensorCalibrationFactor=%4.3f,SensorCalibrationOffset=%4.3f\n-------------\n',1/Est(1),Est(2));

end