function z=MyCost(s,model)

    p=s(1:model.n);
    
    n=model.n;
    w=model.w;
    d=model.d;
    
    z=0;
    for i=1:n
        for j=i+1:n
            z=z+w(i,j)*d(p(i),p(j));
        end
    end

end