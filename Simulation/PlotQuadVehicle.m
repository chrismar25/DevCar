function PlotQuadVehicle(pos_x,pos_y,theta,alpha)


	L=3;
	W=2;
	tire=0.4;
	
	alpha=max(-50*pi/180,alpha);
	alpha=min(50*pi/180,alpha);

	ca=cos(alpha);
	sa=sin(alpha);

	x1=L+sa*(W/2);
	x11 = x1+tire*ca;
	x12 = x1-tire*ca;


	x2=L-(W/2)*sa;
	x21 = x2+tire*ca;
	x22 = x2-tire*ca;
	
	x= [-tire, tire, 0, 0, -tire, tire, 0, 0, L+tire/3, L, x1,x11,x12,x1, x2,x21,x22];
	
	y1=(W/2)*(-ca);
	y11 = y1+tire*sin(alpha);
	y12 = y1-tire*sin(alpha);
	
	y2= ca*(W/2);
	y21 = y2+tire*sa;
	y22 = y2-tire*sa;
	y=[W/2,W/2,W/2,-W/2,-W/2,-W/2,-W/2,0,0,0, y1,y11,y12, y1,y2,y21,y22];
	
	Xv=[x;y;ones(1,17)];
	T=[cos(theta),-sin(theta),pos_x;sin(theta),cos(theta),pos_y;0,0,1];

	Xv=T*Xv;
	plot(Xv(1,:),Xv(2,:),'k-','linewidth',4);
	axis equal
	hold on
