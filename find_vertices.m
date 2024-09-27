function rho =find_vertices(n,x,y,z)
% input are x,y,z it doesnot depend if x is length or width, these are
% entered with respect to the axises 
% centriod of payload coordinates is (0,0) 
  rho=zeros(3,n);

switch n
 case 1
      rho=[0;0;-z/2];
 case 2
     sidelength=min(x,y);
     if x<y
     rho=[-sidelength/2 sidelength/2; 0 0; -z/2*ones(1,n)];
     else
     rho=[ 0 0;-sidelength/2 sidelength/2; -z/2*ones(1,n)];
     end
 case 3
        sidelength=min(x,y);
        h=sqrt(3)/2*sidelength;
        if x<y
            rho=[  sidelength/2 -sidelength/2  0; h/3  h/3 -2/3*h; -z/2 -z/2 -z/2 ]
        else
            rho=[h/3  h/3 -2/3*h;  sidelength/2 -sidelength/2  0; -z/2 -z/2 -z/2 ];  
            % not if x=y any scenario will work as it is symmetric
        end
  case 4  
      rho=[-x/2 -x/2 x/2 x/2;y/2 -y/2 y/2 -y/2; -z/2*ones(1,n)]; %at corners
  otherwise
          interior_angle=(n-2)/n*180;
          sidelength=min(x,y)*cosd(interior_angle/2);
          pgon2 = nsidedpoly(n,'Center',[0 0],'SideLength',sidelength);  
       rho=[pgon2.Vertices';-z/2*ones(1,n)];
end