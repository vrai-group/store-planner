function split_vector = fun(blocks)

    dim_block = size(blocks,1);
    num_blocks = size(blocks,3);
    split_vector = true(1,num_blocks);
    
    for i=1:num_blocks
        if (mean(blocks(:,:,i),'all') >= 230 ) %ok
            split_vector(i) = false;
        end
    end

end

