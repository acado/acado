function [split,cnt]=explode(string,lim)

if isempty(string)
   split{1}='';
   cnt=0;

else 
    r=string;
    i=0;
   
    while ~isempty(r)
        [piece,r]=strtok(remainder,lim);
        i=i+1;
        split{i}=piece;
    end
    cnt=i;
   
end