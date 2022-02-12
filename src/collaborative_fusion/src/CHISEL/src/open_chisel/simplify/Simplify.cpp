#include "Simplify.h"
#define HAS_LABEL 1
namespace chisel{



vec3f barycentric(const vec3f &p, const vec3f &a, const vec3f &b, const vec3f &c){
	vec3f v0 = b-a;
	vec3f v1 = c-a;
	vec3f v2 = p-a;
	double d00 = v0.dot(v0);
	double d01 = v0.dot(v1);
	double d11 = v1.dot(v1);
	double d20 = v2.dot(v0);
	double d21 = v2.dot(v1);
	double denom = d00*d11-d01*d01;
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0 - v - w;
	return vec3f(u,v,w);
}
	void v2Ev( Eigen::Vector3f &v2,vec3f &v1)
	{
		v2(0) = v1.x;
		v2(1) = v1.y;
		v2(2) = v1.z;
	}
vec3f interpolate(const vec3f &p, const vec3f &a, const vec3f &b, const vec3f &c, const vec3f attrs[3])
{
	vec3f bary = barycentric(p,a,b,c);
	vec3f out = vec3f(0,0,0);
	out = out + attrs[0] * bary.x;
	out = out + attrs[1] * bary.y;
	out = out + attrs[2] * bary.z;
	return out;
}

double min(double v1, double v2) {
	return fmin(v1,v2);
}
void Simplifier::simplify_mesh(int target_count, double agressiveness, bool verbose)
	{

		// init
		calculate_error_times = 0;
		loopi(0,triangles.size())
        {
            triangles[i].deleted=0;
        }

		// main iteration loop
		int deleted_triangles=0;
		std::vector<int> deleted0,deleted1;
		int triangle_count=triangles.size();
		int iteration = 0;
		//loop(iteration,0,100)
		for (; iteration < 100; iteration ++)
		{
			if(triangle_count-deleted_triangles<=target_count)break;
			//clock_t start = clock();
			// update mesh once in a while
			if(iteration%5==0)
			{
				update_mesh(iteration);
			}

			// clear dirty flag
			loopi(0,triangles.size()) triangles[i].dirty=0;

			//
			// All triangles with edges below the threshold will be removed
			//
			// The following numbers works well for most models.
			// If it does not, try to adjust the 3 parameters
			//
			double threshold = 0.000000001*pow(double(iteration+3),agressiveness);

			// target number of triangles reached ? Then break
			if ((verbose) && (iteration%5==0)) {
				printf("iteration %d - triangles %d threshold %g\n",iteration,triangle_count-deleted_triangles, threshold);
			}
			double update_triangles_t = 0;
			double check_flip_t = 0;
			// remove vertices & mark deleted triangles
			loopi(0,triangles.size())
			{
				Triangle &t=triangles[i];
				if(t.err[3]>threshold) continue;
				if(t.deleted) continue;
				if(t.dirty) continue;

				loopj(0,3)if(t.err[j]<threshold)
				{

					int i0=t.v[ j     ]; Vertex &v0 = vertices[i0];
					int i1=t.v[(j+1)%3]; Vertex &v1 = vertices[i1];
					// Border check
					if(v0.border != v1.border)  continue;

					// Compute vertex to collapse to
					vec3f p;
					calculate_error(i0,i1,p);
					//clock_t start_t=clock();
					deleted0.resize(v0.tcount); // normals temporarily
					deleted1.resize(v1.tcount); // normals temporarily
					//clock_t end_t=clock();
					//check_flip_t += (double)(end_t-start_t)/CLOCKS_PER_SEC;
					// don't remove if flipped
					if( flipped(p,i0,i1,v0,v1,deleted0) ) continue;

					if( flipped(p,i1,i0,v1,v0,deleted1) ) continue;

					if ( (t.attr & TEXCOORD) == TEXCOORD  )
					{
						update_uvs(i0,v0,p,deleted0);
						update_uvs(i0,v1,p,deleted1);
					}

					// not flipped, so remove edge
					v0.p=p;
					v0.q=v1.q+v0.q;
					int tstart=refs.size();
					//clock_t start=clock();
					update_triangles(i0,v0,deleted0,deleted_triangles);
					update_triangles(i0,v1,deleted1,deleted_triangles);
					//clock_t end=clock();
					//update_triangles_t += (double)(end-start)/CLOCKS_PER_SEC;
					int tcount=refs.size()-tstart;

					if(tcount<=v0.tcount)
					{
						// save ram
						if(tcount)memcpy(&refs[v0.tstart],&refs[tstart],tcount*sizeof(Ref));
					}
					else
						// append
						v0.tstart=tstart;

					v0.tcount=tcount;
					break;
				}
				// done?
				if(triangle_count-deleted_triangles<=target_count)break;
			}
				//clock_t end=clock();

				//std::cout<<"iteration "<<iteration<<": "<<(double)(end-start)/(CLOCKS_PER_SEC)<<std::endl;
				//std::cout<<"update_triangles :"<<update_triangles_t<<std::endl;
				//std::cout<<"check flip :"<<check_flip_t<<std::endl;
		}
		// clean up mesh
		std::cout<<"target triangle number: "<<target_count<<" final triangle number: "<<triangles.size()<<" iteration times: "<<iteration<<std::endl;
		std::cout<<"calculate edge error: "<<calculate_error_times<<std::endl;
		compact_mesh();
		update_mesh(1);
		computeNormal();
	} //simplify_mesh()

	void Simplifier::simplify_mesh_lossless(bool verbose)
	{
		// init
		loopi(0,triangles.size()) triangles[i].deleted=0;

		// main iteration loop
		int deleted_triangles=0;
		std::vector<int> deleted0,deleted1;
		int triangle_count=triangles.size();
		//int iteration = 0;
		//loop(iteration,0,100)
		for (int iteration = 0; iteration < 9999; iteration ++)
		{
			// update mesh constantly
			update_mesh(iteration);
			// clear dirty flag
			loopi(0,triangles.size()) triangles[i].dirty=0;
			//
			// All triangles with edges below the threshold will be removed
			//
			// The following numbers works well for most models.
			// If it does not, try to adjust the 3 parameters
			//
			double threshold = DBL_EPSILON; //1.0E-3 EPS;
			if (verbose) {
				printf("lossless iteration %d\n", iteration);
			}

			// remove vertices & mark deleted triangles
			loopi(0,triangles.size())
			{
				Triangle &t=triangles[i];
				if(t.err[3]>threshold) continue;
				if(t.deleted) continue;
				if(t.dirty) continue;

				loopj(0,3)if(t.err[j]<threshold)
				{
					int i0=t.v[ j     ]; Vertex &v0 = vertices[i0];
					int i1=t.v[(j+1)%3]; Vertex &v1 = vertices[i1];

					// Border check
					if(v0.border != v1.border)  continue;

					// Compute vertex to collapse to
					vec3f p;
					calculate_error(i0,i1,p);

					deleted0.resize(v0.tcount); // normals temporarily
					deleted1.resize(v1.tcount); // normals temporarily

					// don't remove if flipped
					if( flipped(p,i0,i1,v0,v1,deleted0) ) continue;
					if( flipped(p,i1,i0,v1,v0,deleted1) ) continue;

					if ( (t.attr & TEXCOORD) == TEXCOORD )
					{
						update_uvs(i0,v0,p,deleted0);
						update_uvs(i0,v1,p,deleted1);
					}

					// not flipped, so remove edge
					v0.p=p;
					v0.q=v1.q+v0.q;
					int tstart=refs.size();

					update_triangles(i0,v0,deleted0,deleted_triangles);
					update_triangles(i0,v1,deleted1,deleted_triangles);

					int tcount=refs.size()-tstart;

					if(tcount<=v0.tcount)
					{
						// save ram
						if(tcount)memcpy(&refs[v0.tstart],&refs[tstart],tcount*sizeof(Ref));
					}
					else
						// append
						v0.tstart=tstart;

					v0.tcount=tcount;
					break;
				}
			}
			if(deleted_triangles<=0)break;
			deleted_triangles=0;
		} //for each iteration
		// clean up mesh
		compact_mesh();
	} //simplify_mesh_lossless()


	// Check if a triangle flips when this edge is removed
	void Simplifier::computeNormal()
	{
		/*
		compute the triangles normal
		 */
		for(int i = 0;i!= triangles.size();++i)
		{
			int id1 = triangles[i].v[0];
			int id2 = triangles[i].v[1];
			int id3 = triangles[i].v[2];
			vec3f d1 = vertices[id2].p-vertices[id1].p;d1.normalize();
			vec3f d2 = vertices[id3].p-vertices[id1].p;d2.normalize(); 
			triangles[i].n.cross(d1,d2);
			triangles[i].n.normalize();
			//std::cout<<triangles[i].n.x<<" "<<triangles[i].n.y<<" "<<triangles[i].n.z<<std::endl;			
		}
		/*
		compute vertex normal
		 */
		for(int i = 0;i!=vertices.size();++i)
		{
			//std::cout<<normals[i].n.x<<" "<<normals[i].n.y<<" "<<normals[i].n.z<<std::endl;
			Vertex &v = vertices[i];
			vec3f vnormal;//zero std::vector
			loopk(0,v.tcount)
			{
				Triangle &t = triangles[refs[v.tstart + k].tid];
				vnormal += t.n;
			}
			vnormal.normalize();
			normals[i].n = vnormal;
			//std::cout<<normals[i].n.x<<" "<<normals[i].n.y<<" "<<normals[i].n.z<<std::endl;
		}
		std::cout<<"Finish normal computing."<<std::endl;
	}
	bool Simplifier::flipped(vec3f p,int i0,int i1,Vertex &v0,Vertex &v1,std::vector<int> &deleted)
	{

		loopk(0,v0.tcount)
		{
			Triangle &t=triangles[refs[v0.tstart+k].tid];
			if(t.deleted)continue;

			int s=refs[v0.tstart+k].tvertex;
			int id1=t.v[(s+1)%3];
			int id2=t.v[(s+2)%3];

			if(id1==i1 || id2==i1) // delete ?
			{

				deleted[k]=1;
				continue;
			}
			vec3f d1 = vertices[id1].p-p; d1.normalize();
			vec3f d2 = vertices[id2].p-p; d2.normalize();
			if(fabs(d1.dot(d2))>0.999) return true;
			vec3f n;
			n.cross(d1,d2);
			n.normalize();
			deleted[k]=0;
			if(n.dot(t.n)<0.2) return true;
		}
		return false;
	}

    // update_uvs
	void Simplifier::reset()
	{
    triangles.clear();
    vertices.clear();
    _colors.clear();
    normals.clear();
	labels.clear();
    std::vector<_Color> r_colors;
	std::vector<Triangle> r_triangles;
	std::vector<Vertex> r_vertices;
	std::vector<Normal> r_normals;
	std::vector<int> r_labels;
		triangles.swap(r_triangles);
		vertices.swap(r_vertices);
		_colors.swap(r_colors);
		normals.swap(r_normals);
		labels.swap(r_labels);
	}
	void Simplifier::update_uvs(int i0,const Vertex &v,const vec3f &p,std::vector<int> &deleted)
	{
		loopk(0,v.tcount)
		{
			Ref &r=refs[v.tstart+k];
			Triangle &t=triangles[r.tid];
			if(t.deleted)continue;
			if(deleted[k])continue;
			vec3f p1=vertices[t.v[0]].p;
			vec3f p2=vertices[t.v[1]].p;
			vec3f p3=vertices[t.v[2]].p;
			t.uvs[r.tvertex] = interpolate(p,p1,p2,p3,t.uvs);
		}
	}

	// Update triangle connections and edge error after a edge is collapsed

	void Simplifier::update_triangles(int i0,Vertex &v,std::vector<int> &deleted,int &deleted_triangles)
	{
		vec3f p;
		loopk(0,v.tcount)
		{
			Ref &r=refs[v.tstart+k];
			Triangle &t=triangles[r.tid];
			if(t.deleted)continue;
			if(deleted[k])
			{
				t.deleted=1;
				deleted_triangles++;
				continue;
			}
			t.v[r.tvertex]=i0;
			t.dirty=1;
			t.err[0]=calculate_error(t.v[0],t.v[1],p);
			t.err[1]=calculate_error(t.v[1],t.v[2],p);
			t.err[2]=calculate_error(t.v[2],t.v[0],p);
			t.err[3]=min(t.err[0],min(t.err[1],t.err[2]));
			refs.push_back(r);
		}
	}

	// compact triangles, compute edge error and build reference list

	void Simplifier::update_mesh(int iteration)
	{
		if(iteration>0) // compact triangles
		{
			int dst=0;
			loopi(0,triangles.size())
			if(!triangles[i].deleted)
			{
				triangles[dst++]=triangles[i];
			}
			triangles.resize(dst);
		}
		//
		// Init Quadrics by Plane & Edge Errors
		//
		// required at the beginning ( iteration == 0 )
		// recomputing during the simplification is not required,
		// but mostly improves the result for closed meshes
		//
		if( iteration == 0 )
		{
			/*compute Q matrix*/
			clock_t start = clock();
			loopi(0,vertices.size())
			vertices[i].q=SymetricMatrix(0.0);

			loopi(0,triangles.size())
			{
				Triangle &t=triangles[i];
				vec3f n,p[3];
				loopj(0,3) p[j]=vertices[t.v[j]].p;
				n.cross(p[1]-p[0],p[2]-p[0]);
				n.normalize();
				t.n=n;
				loopj(0,3) vertices[t.v[j]].q =
					vertices[t.v[j]].q+SymetricMatrix(n.x,n.y,n.z,-n.dot(p[0]));
			}
			loopi(0,triangles.size())
			{
				// Calc Edge Error
				Triangle &t=triangles[i];vec3f p;
				loopj(0,3) t.err[j]=calculate_error(t.v[j],t.v[(j+1)%3],p);
				t.err[3]=min(t.err[0],min(t.err[1],t.err[2]));
			}
			clock_t end = clock();
			std::cout <<"Compute Q Matrix: "<<(double)(end-start)/(CLOCKS_PER_SEC)<<std::endl;
		}

		// Init Reference ID list
		clock_t start = clock();
		loopi(0,vertices.size())
		{
			vertices[i].tstart=0;
			vertices[i].tcount=0;
		}
		loopi(0,triangles.size())
		{
			Triangle &t=triangles[i];
			loopj(0,3) vertices[t.v[j]].tcount++;
		}
		int tstart=0;
		loopi(0,vertices.size())
		{
			Vertex &v=vertices[i];
			v.tstart=tstart;
			tstart+=v.tcount;
			v.tcount=0;
		}

		// Write References
		refs.resize(triangles.size()*3);
		loopi(0,triangles.size())
		{
			Triangle &t=triangles[i];
			loopj(0,3)
			{
				Vertex &v=vertices[t.v[j]];
				refs[v.tstart+v.tcount].tid=i;
				refs[v.tstart+v.tcount].tvertex=j;
				v.tcount++;
			}
		}
		clock_t end=clock();
		std::cout <<"Update References: "<<(double)(end-start)/(CLOCKS_PER_SEC)<<std::endl;		
		// Identify boundary : vertices[].border=0,1
		if( iteration == 0 )
		{
			std::vector<int> vcount,vids;

			loopi(0,vertices.size())
				vertices[i].border=0;

			loopi(0,vertices.size())
			{
				Vertex &v=vertices[i];
				vcount.clear();
				vids.clear();
				loopj(0,v.tcount)
				{
					int k=refs[v.tstart+j].tid;
					Triangle &t=triangles[k];
					loopk(0,3)
					{
						int ofs=0,id=t.v[k];
						while(ofs<vcount.size())
						{
							if(vids[ofs]==id)break;
							ofs++;
						}
						if(ofs==vcount.size())
						{
							vcount.push_back(1);
							vids.push_back(id);
						}
						else
							vcount[ofs]++;
					}
				}
				loopj(0,vcount.size()) if(vcount[j]==1)
					vertices[vids[j]].border=1;
			}
		}
	}

	// Finally compact mesh before exiting

	void Simplifier::compact_mesh()
	{
		int dst=0;
		loopi(0,vertices.size())
		{
			vertices[i].tcount=0;
		}
		loopi(0,triangles.size())
		if(!triangles[i].deleted)
		{
			Triangle &t=triangles[i];
			triangles[dst++]=t;
			t.deleted = 0;
			loopj(0,3)vertices[t.v[j]].tcount=1;
		}
		triangles.resize(dst);
		dst=0;
		loopi(0,vertices.size())
		if(vertices[i].tcount)
		{
			vertices[i].tstart=dst;
			vertices[dst].p=vertices[i].p;
			normals[dst].n = normals[i].n;
			_colors[dst].c = _colors[i].c;
#if HAS_LABEL
			labels[dst] = labels[i];
#endif
			dst++;
		}
		loopi(0,triangles.size())
		{
			Triangle &t=triangles[i];
			loopj(0,3)t.v[j]=vertices[t.v[j]].tstart;
		}
		//delete the redundant vertices
		vertices.resize(dst);
		_colors.resize(dst);
		normals.resize(dst);
#if HAS_LABEL
		labels.resize(dst);
#endif	
	}

	// Error between vertex and Quadric

	double Simplifier::vertex_error(SymetricMatrix q, double x, double y, double z)
	{
 		return   q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x + q[4]*y*y
 		     + 2*q[5]*y*z + 2*q[6]*y + q[7]*z*z + 2*q[8]*z + q[9];
	}

	// Error for one edge

	double Simplifier::calculate_error(int id_v1, int id_v2, vec3f &p_result)
	{
		// compute interpolated vertex

		SymetricMatrix q = vertices[id_v1].q + vertices[id_v2].q;
		bool   border = vertices[id_v1].border & vertices[id_v2].border;
		double error=0;
		double det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);
		calculate_error_times+=1;
		if ( det != 0 && !border )
		{

			// q_delta is invertible
			p_result.x = -1/det*(q.det(1, 2, 3, 4, 5, 6, 5, 7, 8));	// vx = A41/det(q_delta)
			p_result.y =  1/det*(q.det(0, 2, 3, 1, 5, 6, 2, 7, 8));	// vy = A42/det(q_delta)
			p_result.z = -1/det*(q.det(0, 1, 3, 1, 4, 6, 2, 5, 8));	// vz = A43/det(q_delta)

			error = vertex_error(q, p_result.x, p_result.y, p_result.z);
		}
		else
		{
			// det = 0 -> try to find best result
			vec3f p1=vertices[id_v1].p;
			vec3f p2=vertices[id_v2].p;
			vec3f p3=(p1+p2)/2;
			double error1 = vertex_error(q, p1.x,p1.y,p1.z);
			double error2 = vertex_error(q, p2.x,p2.y,p2.z);
			double error3 = vertex_error(q, p3.x,p3.y,p3.z);
			error = min(error1, min(error2, error3));
			if (error1 == error) p_result=p1;
			if (error2 == error) p_result=p2;
			if (error3 == error) p_result=p3;
		}
		return error;
	}

	char *trimwhitespace(char *str)
	{
		char *end;

		// Trim leading space
		while(isspace((unsigned char)*str)) str++;

		if(*str == 0)  // All spaces?
		return str;

		// Trim trailing space
		end = str + strlen(str) - 1;
		while(end > str && isspace((unsigned char)*end)) end--;

		// Write new null terminator
		*(end+1) = 0;

		return str;
	}

	//Option : MeshPtr mesh
	//
	 size_t Simplifier:: load_mesh(MeshPtr &mesh)
	{
			Vertex v;

			Normal n;
			_Color c;
			Triangle t;
		for(int i =0;i!=mesh->vertices.size();i+=3)
		{
			for(int j = 0;j!=3;++j)
			{
			v.p = mesh->vertices[i+j];
			n.n = mesh-> normals[i+j];
			c.c = mesh->colors[i+j];
			vertices.push_back(v);
			normals.push_back(n);
			_colors.push_back(c);
#if HAS_LABEL
			labels.push_back(mesh->labels[i+j]);
#endif
			}
			t.v[0] = i;
			t.v[1] = i+1;
			t.v[2] = i+2;
			triangles.push_back(t);
		}
		return triangles.size();

	}
    inline size_t Simplifier:: load_mesh_from_compact(MeshPtr &mesh)
    {
			Vertex v;
			Normal n;
			_Color c;
			Triangle t;
			int label;
		for(int i =0;i!=mesh->compact_vertices.size();++i)
		{
			v.p = mesh->compact_vertices[i];
			n.n = mesh->compact_normals[i];
			c.c = mesh->compact_colors[i];
		
			vertices.push_back(v);
			normals.push_back(n);
			_colors.push_back(c);
#if HAS_LABEL
			labels.push_back(mesh->compact_labels[i]);
#endif
        }
			for(int i = 0;i!=mesh->triangles.size();++i)
			{
                t.v[0] = mesh->triangles[i](0);
                t.v[1] = mesh->triangles[i](1);
                t.v[2] = mesh->triangles[i](2);

                triangles.push_back(t);
            }
        return triangles.size();
		
    }

	inline size_t Simplifier:: save_mesh(MeshPtr &mesh)
	{
		mesh->vertices.resize(triangles.size()*3);
		mesh->colors.resize(triangles.size()*3);
#if HAS_LABEL
		mesh->labels.resize(triangles.size()*3);
#endif
		mesh->normals.resize(triangles.size()*3);
		mesh->indices.resize(triangles.size()*3);
		for(int i = 0;i!=triangles.size();++i)
		{
			Triangle t = triangles[i];
			for(int j = 0;j!=3;++j)
			{
			v2Ev(mesh->vertices[i*3+j] , vertices[t.v[j]].p);
			v2Ev(mesh->colors[i*3+j] ,_colors[t.v[j]].c);
			v2Ev(mesh->normals[i*3+j] , normals[t.v[j]].n);
#if HAS_LABEL
			mesh->labels[i*3+j] = labels[t.v[j]];
#endif
			mesh->indices[i*3+j] = i*3+j;

			}
		}

		return triangles.size();
	}
    inline size_t Simplifier::save_mesh_to_compact(MeshPtr &mesh)
    {
        mesh->compact_vertices.resize(vertices.size());
        mesh->compact_colors.resize(_colors.size());
#if HAS_LABEL
		mesh->compact_labels.resize(labels.size());
#endif
        mesh->compact_normals.resize(normals.size());
        mesh->compact_indices.clear();
        mesh->triangles.resize(triangles.size());
        for(int i = 0;i!=vertices.size();++i)
        {
            v2Ev(mesh->compact_vertices[i],vertices[i].p);
            v2Ev(mesh->compact_colors[i],_colors[i].c);
            v2Ev(mesh->compact_normals[i],normals[i].n);
            mesh->compact_indices[mesh->compact_vertices[i]] = i;
#if HAS_LABEL
			mesh->compact_labels[i] = labels[i];
#endif
        }
        for(int i =0;i!=triangles.size();++i)
        {
            mesh->triangles[i] = Point3(triangles[i].v[0],triangles[i].v[1],triangles[i].v[2]);
        }
        return triangles.size();
    }
	void Simplifier::simplify_mesh_from_CHISEL_compact(MeshPtr &m,float compact_ratio)
    { 
        reset();
#if HAS_LABEL
		std::cout<<"simplify mesh with semantic!"<<std::endl;
#endif
		std::cout<<"labels: "<<m->compact_labels.size()<<" vertices: "<<m->compact_vertices.size()<<" normals: "<<m->compact_normals.size()<<std::endl;
		size_t size = load_mesh_from_compact(m);
		//std::cout<<c<<std::endl;

		//write_obj(("/media/wlsdzyzl/wlsdzyzl_1/models/_"+std::to_string(submapID)+".obj").c_str());
        //submapID+=1;
		//printf("Old Size: %d,should be reduced
        //if(compact_ratio>=1) return;
        if(compact_ratio<=0.0)
        {
            m->clearCompact();
            m->clearNotCompact();
            return; 
        }
		simplify_mesh(size*compact_ratio);
		std::cout<<"Start to clear simplified meshes."<<std::endl;
        m->clearCompact();
        m->clearNotCompact();
		std::cout<<"Save meshes to CHISEL mesh."<<std::endl;
        size = save_mesh(m);//_to_compact(m);
		std::cout<<"After simplifying: "<<size<<std::endl;
        //m->clearCompact();    
        reset();
        //printf("New Size: %d\n\n",size);
    }
    void Simplifier::compact_to_no(MeshPtr &m)
    {
        reset();
		size_t size = load_mesh_from_compact(m);
        save_mesh(m);     
        reset();
    }
	void Simplifier::simplify_mesh_from_CHISEL(MeshPtr &m,float compact_ratio)
	{
		reset();
		size_t size = load_mesh(m);
        printf("Old Size: %d,should be reduced to %lf\n",size,size/compact_ratio);
        //printf("Simplifying:******************************************************************************************\n");
        if(compact_ratio < 1.0) 
		simplify_mesh(size*compact_ratio);
		size=save_mesh(m);
        printf("New Size: %d\n\n",size);
	}
	/*
	void load_obj(const char* filename, bool process_uv=false){
		vertices.clear();
		triangles.clear();
		normals.clear();
		//printf ( "Loading Objects %s ... \n",filename);
		FILE* fn;
		if(filename==NULL)		return ;
		if((char)filename[0]==0)	return ;
		if ((fn = fopen(filename, "rb")) == NULL)
		{
			printf ( "File %s not found!\n" ,filename );
			return;
		}
		char line[1000];
		memset ( line,0,1000 );
		int vertex_cnt = 0;
		int material = -1;
		std::map<std::string, int> material_map;
		std::vector<vec3f> uvs;
		std::vector<std::vector<int> > uvMap;

		while(fgets( line, 1000, fn ) != NULL)
		{
			Vertex v;
			vec3f uv;
			Normal n;
			Color c;

			if (strncmp(line, "mtllib", 6) == 0)
			{
				mtllib = trimwhitespace(&line[7]);
			}
			if (strncmp(line, "usemtl", 6) == 0)
			{
				std::string usemtl = trimwhitespace(&line[7]);
				if (material_map.find(usemtl) == material_map.end())
				{
					material_map[usemtl] = materials.size();
					materials.push_back(usemtl);
				}
				material = material_map[usemtl];
			}

			if ( line[0] == 'v' && line[1] == 't' )
			{
				if ( line[2] == ' ' )
				if(sscanf(line,"vt %lf %lf",
					&uv.x,&uv.y)==2)
				{
					uv.z = 0;
					uvs.push_back(uv);
				} else
				if(sscanf(line,"vt %lf %lf %lf",
					&uv.x,&uv.y,&uv.z)==3)
				{
					uvs.push_back(uv);
				}
			}
			else if(line[0] == 'v' && line[1] == 'n')
			{
				if( line[2] ==' ')
				if(sscanf(line,"vn %lf %lf %lf", &n.n.x, &n.n.y,&n.n.z) == 3)
				{
					normals.push_bacl(v);
				}
			}
			else if ( line[0] == 'v' )
			{
				if ( line[1] == ' ' )
				if(sscanf(line,"v %lf %lf %lf",
					&v.p.x,	&v.p.y,	&v.p.z)==3)
				{
					vertices.push_back(v);
				}
			}
			int integers[9];
			if ( line[0] == 'f' )
			{
				Triangle t;
				bool tri_ok = false;
                bool has_uv = false;

				if(sscanf(line,"f %d %d %d",
					&integers[0],&integers[1],&integers[2])==3)
				{
					tri_ok = true;
				}else
				if(sscanf(line,"f %d// %d// %d//",
					&integers[0],&integers[1],&integers[2])==3)
				{
					tri_ok = true;
				}else
				if(sscanf(line,"f %d//%d %d//%d %d//%d",
					&integers[0],&integers[3],
					&integers[1],&integers[4],
					&integers[2],&integers[5])==6)
				{
					tri_ok = true;
				}else
				if(sscanf(line,"f %d/%d/%d %d/%d/%d %d/%d/%d",
					&integers[0],&integers[6],&integers[3],
					&integers[1],&integers[7],&integers[4],
					&integers[2],&integers[8],&integers[5])==9)
				{
					tri_ok = true;
					has_uv = true;
				}
				else
				{
					printf("unrecognized sequence\n");
					printf("%s\n",line);
					while(1);
				}
				if ( tri_ok )
				{
					t.v[0] = integers[0]-1-vertex_cnt;
					t.v[1] = integers[1]-1-vertex_cnt;
					t.v[2] = integers[2]-1-vertex_cnt;
					t.attr = 0;

					if ( process_uv && has_uv )
					{
						std::vector<int> indices;
						indices.push_back(integers[6]-1-vertex_cnt);
						indices.push_back(integers[7]-1-vertex_cnt);
						indices.push_back(integers[8]-1-vertex_cnt);
						uvMap.push_back(indices);
						t.attr |= TEXCOORD;
					}

					t.material = material;
					//geo.triangles.push_back ( tri );
					triangles.push_back(t);
					//state_before = state;
					//state ='f';
				}
			}
		}

		if ( process_uv && uvs.size() )
		{
			loopi(0,triangles.size())
			{
				loopj(0,3)
				triangles[i].uvs[j] = uvs[uvMap[i][j]];
			}
		}

		fclose(fn);

		//printf("load_obj: vertices = %lu, triangles = %lu, uvs = %lu\n", vertices.size(), triangles.size(), uvs.size() );
	} // load_obj()
	*/
	// Optional : Store as OBJ

	void Simplifier::write_obj(const char* filename)
	{
		
		FILE *file=fopen(filename, "w");
		int cur_material = -1;
		bool has_uv = false;//(triangles.size() && (triangles[0].attr & TEXCOORD) == TEXCOORD);

		if (!file)
		{
			printf("write_obj: can't write data file \"%s\".\n", filename);
			exit(0);
		}
		if (!mtllib.empty())
		{
			fprintf(file, "mtllib %s\n", mtllib.c_str());
		}
		loopi(0,vertices.size())
		{
			//fprintf(file, "v %lf %lf %lf\n", vertices[i].p.x,vertices[i].p.y,vertices[i].p.z);
			fprintf(file, "v %g %g %g\n", vertices[i].p.x,vertices[i].p.y,vertices[i].p.z); //more compact: remove trailing zeros
		}
		if (has_uv)
		{
			loopi(0,triangles.size()) if(!triangles[i].deleted)
			{
				fprintf(file, "vt %g %g\n", triangles[i].uvs[0].x, triangles[i].uvs[0].y);
				fprintf(file, "vt %g %g\n", triangles[i].uvs[1].x, triangles[i].uvs[1].y);
				fprintf(file, "vt %g %g\n", triangles[i].uvs[2].x, triangles[i].uvs[2].y);
			}
		}
		int uv = 1;
		loopi(0,triangles.size()) 
		{
			triangles[i].deleted = 0;
			if(!triangles[i].deleted)
		{
			
			{
				fprintf(file, "f %d %d %d\n", triangles[i].v[0]+1, triangles[i].v[1]+1, triangles[i].v[2]+1);
			}
			//fprintf(file, "f %d// %d// %d//\n", triangles[i].v[0]+1, triangles[i].v[1]+1, triangles[i].v[2]+1); //more compact: remove trailing zeros
		}
		}
		fclose(file);
	}
};