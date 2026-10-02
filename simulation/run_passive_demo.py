"""Independent Python execution of basic passive classroom physics models."""
from pathlib import Path
import json
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parents[1]/'results'/'python'
OUT.mkdir(parents=True,exist_ok=True)
g=9.81; m=.005; rho=1.225; cd=.47; radius=.035; h=1.
c=.5*rho*cd*np.pi*radius**2/m
def ground(t,x): return x[0]
ground.terminal=True
ground.direction=-1
def drop(drag,step):
    return solve_ivp(lambda t,x:[x[1],-g-drag*abs(x[1])*x[1]],(0,3),[h,0],events=ground,rtol=1e-9,atol=1e-11,max_step=step)
s=drop(c,.01); t=s.t; z,v=s.y
exact=h-np.log(np.cosh(np.sqrt(g*c)*t))/c
err=float(np.max(abs(z-exact)))
energy=m*g*z+.5*m*v*v
vac=drop(0,.01)
vacerr=abs(vac.t[-1]-np.sqrt(2*h/g))
stepdiff=abs(t[-1]-drop(c,.005).t[-1])
L=.3; mp=.02; b=.001; J=mp*L*L
p=solve_ivp(lambda t,x:[x[1],-g/L*np.sin(x[0])-b/J*x[1]],(0,12),[np.deg2rad(15),0],t_eval=np.linspace(0,12,1201),rtol=1e-9,atol=1e-11)
ep=.5*J*p.y[1]**2+mp*g*L*(1-np.cos(p.y[0]))
assert err<1e-7 and vacerr<1e-7 and stepdiff<1e-7
assert np.max(np.diff(energy))<1e-9 and np.max(np.diff(ep))<1e-9
np.savetxt(OUT/'drop.csv',np.column_stack([t,z,v,energy]),delimiter=',',header='time_s,height_m,velocity_m_s,energy_J',comments='')
np.savetxt(OUT/'pendulum.csv',np.column_stack([p.t,p.y.T,ep]),delimiter=',',header='time_s,angle_rad,rate_rad_s,energy_J',comments='')
metrics=dict(execution='Python SciPy; MATLAB not executed',drop_time_s=float(t[-1]),analytic_max_height_error_m=err,vacuum_time_error_s=float(vacerr),max_step_halving_time_difference_s=float(stepdiff),drop_energy_max_increment_J=float(np.max(np.diff(energy))),pendulum_energy_max_increment_J=float(np.max(np.diff(ep))),checks='PASS')
(OUT/'validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig,ax=plt.subplots(2,2,figsize=(10,6),layout='constrained')
for a,tx,yy,title,unit in [(ax[0,0],t,z,'Soft sphere vertical drop','Height (m)'),(ax[0,1],t,v,'Vertical speed','Velocity (m/s)'),(ax[1,0],p.t,np.rad2deg(p.y[0]),'Fixed-pivot passive pendulum','Angle (deg)'),(ax[1,1],p.t,ep,'Pendulum energy','Energy (J)')]:
    a.plot(tx,yy,color='#226484',lw=2); a.set(title=title,xlabel='Time (s)',ylabel=unit); a.grid(alpha=.2)
fig.savefig(OUT/'passive_demo.png',dpi=180); plt.close(fig)
print(json.dumps(metrics,indent=2))
