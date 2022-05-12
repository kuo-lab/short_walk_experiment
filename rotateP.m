function [Prot] = rotateP(P,r,axis)
  
  if strcmp(axis,'x')
    rot = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];      
  end
  if strcmp(axis,'y')
    rot = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];   
  end
  if strcmp(axis,'z')
    rot = [cos(r) -sin(r) 0; sin(r) cos(r) 0; 0 0 1];      
  end
  Prot = P*rot;

end
