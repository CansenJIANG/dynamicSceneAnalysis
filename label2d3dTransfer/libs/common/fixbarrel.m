% function to convert the fisheye image to real-scale rectangle linear image 
function [x1,y1]=fixbarrel(in)

%[x,y]=fixbarrel(image)
%%Takes an image and returns the x and y coordinates that transform the
%%image to rectilinear points from fisheye.


[row,col,fghfgh]=size(in);

x0=(zeros(row,1)+1)*(1:col)-(col+1)/2    ;   %%define x coordinates
y0=((1:row)'*(zeros(1,col)+1))-(row+1)/2   ; %%define y coordinates
d=.9*col;                   %%"focal length" dependent on lens and resolution
sc=.6;                      %%scale factor, fiddle with this

%%optimizations
d2=1/d;                      %avoid division
k0=sc*d;                    %avoid remultiplying constants
k1=sqrt(x0.^2+y0.^2);       %intermediate stuff
k2=k0./k1;                  %consolidate reciprocals
k1=k1.*d2;                  %consolidate constant multiples while avoiding memory access
%%snoitazimitpo

x1=tan(k1).*x0.*k2;             %%magic happens
y1=tan(k1).*y0.*k2;             %%more magic
%%  plot(x1(:),y1(:),'+'); plot output grid for testing
%%x1=x1.*sc; optimized out
%%y1=y1.*sc; optimized out
y1=round(y1);                   %descretize
x1=round(x1);                   %descretize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Done. The rest is just for displaying, it should be commented out in
%%actual useage. x1 and y1 provide coordinates relative to the optic axis
%%data still needs to be fed to the planar perspective transform to map
%%onto the ground plane.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
xmax=max(abs(x1(:)));           %find output bounds
ymax=max(abs(y1(:)));

drwout=zeros(2*ymax+1,2*xmax+1,3,'uint8')+255;   %prepare an output image

addx=xmax+1;    %consolidate addition
addy=ymax+1;

x1=x1+addx;     %add and store back to save memory (invalidates output!!!!)
y1=y1+addy;
for irow= 1:row
    for icol=1:col
        drwout(y1(irow,icol),x1(irow,icol),:)=in(irow,icol,:); %write to output image
    end
end


image(drwout);

%%max(abs(sqrt(x0(:).^2+y0(:).^2)/d))  old check for valid parameters