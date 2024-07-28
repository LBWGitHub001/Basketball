#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt8.h>

#include <ComSerial.h>

#include <iostream>
#include <chrono>
#include <cmath>

using namespace nvinfer1;

static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

int count_= 0;
image_transport::Publisher Pub;
struct Result_buffer{
    int x;
    int y;
    int distance;
    int height;
    float confidence;
};

ComSerial com("/dev/ttyUSB0",115200,100);

ros::Publisher boxPub;

Result_buffer result_buffer[4] = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
};

int direction_buffer = -1;
int aim_zero_count = 0;

cv::Mat detected_src;
cv::Mat origin_src;

float* gpu_buffers[2];
cudaStream_t stream;
IRuntime* runtime = nullptr;
ICudaEngine* engine = nullptr;
IExecutionContext* context = nullptr;
float* cpu_output_buffer = nullptr;

uint8_t aim = 0;


void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) {
    context.enqueue(batchsize, gpu_buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw, std::string& wts_name, std::string& engine_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6) {
        engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    } else {
        engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    IHostMemory* serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    // Save engine to file
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cerr << "Could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
    serialized_engine->destroy();
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(src);

    // Preprocess
    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Run inference
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    draw_bbox(img_batch, res_batch);
    for(auto& iter : img_batch){
        sensor_msgs::ImagePtr msg1;
        msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", iter).toImageMsg();
        Pub.publish(msg1);
    }

//            cv::imshow("dews",src);
    for(int i = 0;i < img_batch.size();i++){
        auto& img = img_batch[i];
        auto& res = res_batch[i];
        com.sendPort(img,res);
    }
    // Draw bounding boxes and calc distance height
   /*
    calc_distance(img_batch, res_batch);
    int flag=0;
    ROS_INFO_STREAM("*****************************************");
    for(auto& it1 : res_batch){
        for(auto& it2 : it1){
            if(it2.class_id==1){
                ROS_INFO_STREAM("id:\t" << it2.class_id);
                ROS_INFO_STREAM("conf:\t" << it2.conf);

                flag = 1;
            }

        }
    }
    if(flag==0)
        ROS_FATAL_STREAM("No Aim");
*/
}

int main(int argc, char** argv) {
    cudaSetDevice(kGpuId);

    ros::init(argc, argv, "yolov5");
    ros::NodeHandle nh;
    image_transport::ImageTransport it2(nh);
    Pub = it2.advertise("Result", 1);
    std::string wts_name;
    std::string engine_name;
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;

    char s_or_d = 'd';

    if (s_or_d == 'd'){
        engine_name = "/home/lbw/BasketBall/1111Code/yolov5/engine/best1.engine";
    } else if (s_or_d == 's'){
        wts_name = "/home/lbw/BasketBall/1111Code/yolov5/weights/All.wts";
        engine_name = "/home/lbw/BasketBall/1111Code/yolov5/engine/All.engine";
        gd = 0.33;
        gw = 0.50;
        is_p6 = false;
    }

    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
        serialize_engine(kBatchSize, is_p6, gd, gw, wts_name, engine_name);
        return 0;
    }

    // Deserialize the engine from file

    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("Image_Handle", 1, &imageCallback);
    image_transport::Publisher detectedImgPub = it.advertise("/detected_img", 1);

//    ros::Subscriber paramSub = nh.subscribe("/param", 1, &paramCallback);


    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

    }


    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cuda_preprocess_destroy();
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();

    // Print histogram of the output distribution
    // std::cout << "\nOutput:\n\n";
    // for (unsigned int i = 0; i < kOutputSize; i++) {
    //   std::cout << prob[i] << ", ";
    //   if (i % 10 == 0) std::cout << std::endl;
    // }
    // std::cout << std::endl;

    return 0;
}
