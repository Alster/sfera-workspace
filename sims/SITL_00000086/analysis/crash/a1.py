from u import *
for w in [(25,65),(69.4,73.2),(73.6,77.2),(77.2,79.8)]:
    print('==',w)
    stats('RCOU',['C1','C2','C3','C4'],*w)
    stats('MOTB',['ThLimit','ThrAvMx','ThrOut','LiftMax','FailFlags'],*w)
    stats('QTUN',['ThO','ThH','TMix','Trn','Ast'],*w)
    stats('CTUN',['ThO','ThD','As','SAs','AsT'],*w)
    stats('BAT',['Volt','Curr'],*w)
