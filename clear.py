from dxl.dxlchain import DxlChain
from dxl.dxlcore import Dxl

port="COM21"
timeout=0.01

rates=[2000000/(data+1) for data in range(0,255)]
rates.append(2250000)
rates.append(2500000)
rates.append(3000000)

    
chain=DxlChain(port,rate=1000000,timeout=timeout)
for rate in rates:
    print "rate",rate
    chain.reopen(port,rate,timeout=timeout)
    try:
        chain.factory_reset(Dxl.BROADCAST)
    except Exception,e:
        print e
    chain.close()


    