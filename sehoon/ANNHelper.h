#ifndef SEHOON_ANN_HELPER_H
#define SEHOON_ANN_HELPER_H

#include <vector>
#include <glog/logging.h>
#include <ANN/ANN.h>
#include <Eigen/Dense>

#include "Moreeigen.h"

namespace sehoon {
    namespace ann {

        class KDTree;

        class KDTree {
        public:
			KDTree() : dim(0), kdTree(NULL)
			{

			}
            KDTree(int _dim)
                : dim(_dim), kdTree(NULL) {
            }
            virtual ~KDTree() {
                delete kdTree;
            }
			void clean()
			{
				if (kdTree) delete kdTree;
				kdTree = NULL;
				prePts.clear();
			}
			void setDim(int _dim) { dim = _dim; }
            int getDIM() const { return dim; }
    
            void add(const Eigen::VectorXf& pt) {
                CHECK_EQ(pt.size(), dim);
                prePts.push_back(pt);
            }
            
            void initANN() {
                int N = prePts.size();
                dataPts = annAllocPts(N, dim);

                for (int i = 0; i < N; i++) {
                    const Eigen::VectorXf& pt( prePts[i] );
                    ANNpoint p = dataPts[i];
                    for (int j = 0; j < dim; j++) {
                        p[j] = pt(j);
                    }
                }

                kdTree = new ANNkd_tree(dataPts, N, dim);
            }
    
            std::vector<int> kSearch(const Eigen::VectorXf& pt, int k, bool V = false) {
                ANNpoint queryPt = annAllocPt(dim);
                for (int i = 0; i < dim; i++) {
                    queryPt[i] = pt(i);
                }
    
                ANNidxArray idx = new ANNidx[k];
                ANNdistArray dist = new ANNdist[k];
                kdTree->annkSearch(queryPt, k, idx, dist, 0.0);

                vector<int> ret;
                ret.resize(k);
                for (int i = 0; i < k; i++) {
                    ret[i] = idx[i];
                }

                //if (V) {
                //    using google::INFO;
                //    using sehoon::moreeigen::IO;
                //    
                //    LOG(INFO) << "query = " << IO(pt);
                //    for (int i = 0; i < k; i++) {
                //        LOG(INFO) << i << " "
                //                  << dist[i] << " "
                //                  << ret[i] << " "
                //                  << IO(prePts[ret[i]]);
                //    }
                //}

                delete[] idx;
                delete[] dist;
                return ret;                
            }
            
        public:
            int dim;
            std::vector<Eigen::VectorXf> prePts;
            ANNpointArray dataPts;
            ANNkd_tree* kdTree;
        };


    } // namespace ann
} // namespace sehoon

#endif
