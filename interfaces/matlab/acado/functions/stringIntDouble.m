function r = stringIntDouble(val)


    if (isa(val,'numeric'))
        if (val == round(val))
            r = sprintf('%d', val);
        else
            r = sprintf('%E', val);  
        end
    else
        r = val;
    end


end