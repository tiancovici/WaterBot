vector<particle> resample( vector<particle> oldParticles, vector<f64_t> cdf_arr)
{
   f64_t randVal;
   vector<particle> newParticles(oldParticles);
   vector<particle>::iterator oldP_it, newP_it;
   vector<f64_t>::iterator cdf_it;
   
   /* Re-sample every particle */
   for(newP_it = newParticles.begin(); newP_it != newParticles.end(); newP_it++)
   {
      randVal = ((f64_t)(rand()%1000))/1000.0;
      /* Linear search CDF array for new particle */
      for(cdf_it = cdf_arr.begin(), oldP_it = oldParticles.begin(); cdf_it != cdf_arr.end(); )
      {
         if(randVal > *cdf_it)
         {
            cdf_it++; oldP_it++;
         }
         else
         {
            *newP_it = *oldP_it;
            break;
         }
      }
   }
   return newParticles;
};

vector<f64_t> norm_cdf_vector(vector<particle> arr){
   f64_t sum;
   vector<particle>::iterator in_it;   /* Double iterator*/
   vector<f64_t> cdf_arr;

   /* Normalize Vector */
   for(sum = 0.0, in_it = arr.begin(); in_it != arr.end(); in_it++)
      sum += in_it->w;

   for(in_it = arr.begin(); in_it < arr.end(); in_it++)
      in_it->w /=sum;

   /* Create CDF Vector */
   for(sum=0.0, in_it = arr.begin(); in_it < arr.end(); in_it++)
   {
      sum+= in_it->w;
      cdf_arr.push_back(sum);
   }
   return cdf_arr;
}


void main(void)
{
   srand(7);   /* Plant a seed for random functions*/
   u32_t i;
   vector<f64_t>  weights_cdf;
   vector<particle>::iterator it;   /* Particle iterator*/
   vector<particle> particles;

   /* Create Particles */
   for(i=0; i < 10; i++)
      particles.push_back(particle());


   weights_cdf = norm_cdf_vector(particles);
   particles = resample(particles, weights_cdf); 
   /* Clear weights */
   for(it = particles.begin(); it != particles.end(); it++)
      it->w = 0;
}