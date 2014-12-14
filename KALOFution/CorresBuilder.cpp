#include "CorresBuilder.h"
#include "DataProvider.h"

#include <pcl/common/common.h>
#include <pcl/console/print.h>

float CorresBuilder::volumeOverlapRatio(const Eigen::Affine3f& trans)
{
    const int res = 20;
    double ul = 1. / res;
    int s = 0;
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < res; k++) {
                Eigen::Vector3f pos((i + 0.5) * ul, (j + 0.5) * ul, (k + 0.5) * ul, 1);
                Eigen::Vector3f ppos = trans * pos;
                if (ppos(0) >= 0 && ppos(0) <= 1. &&
                    ppos(1) >= 0 && ppos(1) <= 1. &&
                    ppos(2) >= 0 && ppos(2) <= 1.) {
                    s++;
                }
            }
        }
    }

    return (double)s / res / res / res;
}

void CorresBuilder::operator()(DataProvider& provider)
{
    assert(provider.size() >= 2);
    PCL_INFO("Start build correspondence...\n");

    m_pointClouds.resize(provider.size());

    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        CloudTypePtr rawCloud = provider[i];
        CloudTypePtr cloud(new CloudType);
        for (int j = 0; j < rawCloud->points.size(); ++j)
        {
            if (!isnan(rawCloud->points[j].normal_x)) {
                cloud->push_back(rawCloud->points[j]);
            }
        }
        m_pointClouds[i] = cloud;
    }

    m_initCloudTransform.resize(provider.size());

    Eigen::Affine3f base = provider.initTransformOfCloudAtIndex(0).matrix().inverse();
    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        Eigen::Affine3f trans = provider.initTransformOfCloudAtIndex(i);
        m_initCloudTransform[i] = trans;
    }

    for (uint32_t i = 0; i < provider.size() - 1; ++i)
    {
        m_cloudPairs.push_back(CloudPair(i, i + 1, (CloudTransform)(m_initCloudTransform[i].matrix().inverse() * m_initCloudTransform[i + 1].matrix())));
        for (uint32_t j = i + 2; j < provider.size(); ++j)
        {
            CloudTransform trans(m_initCloudTransform[i].matrix().inverse() * m_initCloudTransform[j].matrix());
            if (volumeOverlapRatio(trans) > 0.3f) {
                m_cloudPairs.push_back(CloudPair(i, j, trans));
            }
        }
    }
    PCL_WARN("%d cloud pairs\n", m_cloudPairs.size());
}
