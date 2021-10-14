%% This function is used to find the quadtree decomposition of the heatmap.
% it calculates both hotmap and coldmap so it has to be run just 1 time
% once the desired split level is obtained

function quadtree_decomp(folder_name)

	if ( folder_name == "" )
		disp('Please provide folder name as argument.')	
		exit(-1)
	end

	I_hotmap = imread(strcat('../heatmaps/',folder_name ,'/hotmap.png'));
	I_coldmap = imread(strcat('../heatmaps/',folder_name, '/coldmap.png'));

	if ( size(I_hotmap,1) == 0 ) || ( size(I_coldmap,1) == 0)
		disp('Hotmap and Coldmap not found. Check folder name.')	
		exit(-2)
	end

	min_blk_size_ = 8;
	max_blk_size_ = 64;

	%both images have same size
	max_ = max(size(I_hotmap,1),size(I_hotmap,2));
	min_ = min(size(I_hotmap,1),size(I_hotmap,2));

	nearest_ = pow2(ceil(log2(max_)));

	hot_I = I_hotmap;
	cold_I = I_coldmap;

	%resize the image to be multiple of 2
	if (size(I_hotmap,1) > size(I_hotmap,2))
	    hot_I(:,size(I_hotmap,2):max_) = 0;
	    cold_I(:,size(I_hotmap,2):max_) = 0;
	end

	if (size(I_hotmap,2) > size(I_hotmap,1))
	    hot_I(size(I_hotmap,2):max_,:) = 0;
	    cold_I(size(I_hotmap,2):max_,:) = 0;
	    
	end

	if (nearest_ ~= max_)
	    hot_I(max_:nearest_,max_:nearest_) = 0;
	    cold_I(max_:nearest_,max_:nearest_) = 0;
	end


	threshold = 0.25;
	%function handler splits until all the pixels on the block are white
	hot_S = qtdecomp(hot_I,@fun);
	cold_S = qtdecomp(cold_I,@fun);

	hot_blocks = repmat(uint8(0),size(hot_S));
	cold_blocks = repmat(uint8(0),size(hot_S));

	resized_hot_S = full(hot_S(1:size(I_hotmap,1),1:size(I_hotmap,2)));
	resized_cold_S = full(cold_S(1:size(I_hotmap,1),1:size(I_hotmap,2)));

	mkdir(strcat('../config/', folder_name,'/quadtree_blocks/hotmap/'));
	mkdir(strcat('../config/', folder_name,'/quadtree_blocks/coldmap/'));

	delete(strcat('../config/', folder_name,'/quadtree_blocks/hotmap/*'))
	delete(strcat('../config/', folder_name,'/quadtree_blocks/coldmap/*'))

	dim = nearest_;
	% for dim = [1024 512 256 128 64 32 16 8 4 2 1]   
	while dim >=1
	    
	  num_hotblocks = length(find(hot_S==dim));    
	  if (num_hotblocks > 0)        
	    values = repmat(uint8(1),[dim dim num_hotblocks]);
	    values(2:dim,2:dim,:) = 0;
	    hot_blocks = qtsetblk(hot_blocks,hot_S,dim,values);
	  end

	  %write to file hotblocks
	  [r,c] = find(resized_hot_S==dim);
	  fid=fopen(strcat('../config/', folder_name,'/quadtree_blocks/hotmap/',num2str(dim),'.txt'),'w');
	  for i=1:length(r)
	    fprintf(fid, '%d %d\n', c(i),r(i));
	  end
	  fclose(fid);
	  
	  num_coldblocks = length(find(cold_S==dim));    
	  if (num_coldblocks > 0)        
	    values = repmat(uint8(1),[dim dim num_coldblocks]);
	    values(2:dim,2:dim,:) = 0;
	    cold_blocks = qtsetblk(cold_blocks,cold_S,dim,values);
	  end

	  %write to file coldblocks
	  [r,c] = find(resized_cold_S==dim);
	  fid=fopen(strcat('../config/', folder_name,'/quadtree_blocks/coldmap/',num2str(dim),'.txt'),'w');
	  for i=1:length(r)
	    fprintf(fid, '%d %d\n', c(i),r(i));
	  end
	  fclose(fid);
	  
	  dim = dim/2;
	  
	end

	exit(0)
end
