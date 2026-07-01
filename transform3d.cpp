// Transform3D.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include<fstream>
#include<stdlib.h>

#include<vtkSmartPointer.h>
#include<vtkDICOMImageReader.h>
#include<vtkImageViewer2.h>
#include<vtkImageActor.h>
#include<vtkRenderer.h>
#include<vtkRenderWindow.h>
#include<vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include<vtkImageThreshold.h>
#include<vtkObjectFactory.h>
#include<vtkRendererCollection.h>
#include<vtkLookupTable.h>
#include<vtkImageMapToColors.h>
#include<vtkSliderWidget.h>
#include<vtkSliderRepresentation2D.h>
#include<vtkImageViewer2.h>
#include<vtkProperty2D.h>
#include<vtkTextProperty.h>
#include<vtkImageProperty.h>

#include<itkImage.h>
#include <itkImageFileReader.h>
#include<itkGDCMSeriesFileNames.h>
#include<itkNumericSeriesFileNames.h>
#include<itkImageSeriesReader.h>
#include<itkImageSeriesWriter.h>
#include "itkGDCMImageIO.h"
#include<itkBinaryThresholdImageFilter.h>
#include<itkConnectedThresholdImageFilter.h>
#include<itkCurvatureFlowImageFilter.h>
#include<itkImageRegionIterator.h>
#include<itkImageLinearIteratorWithIndex.h>
#include<itkNeighborhoodAlgorithm.h>
#include<itkNeighborhoodIterator.h>
#include<itkImageSliceIteratorWithIndex.h>
#include<itkSliceBySliceImageFilter.h>
#include<itkExtractImageFilter.h>
#include<itkJoinImageFilter.h>
#include<itkJoinSeriesImageFilter.h>
#include<itkHausdorffDistanceImageFilter.h>

#include "vtkProperty.h"
#include<vtkPiecewiseFunction.h>
#include<vtkGPUVolumeRayCastMapper.h>
#include<vtkVolumeProperty.h>
#include<vtkColorTransferFunction.h>
#include<vtkLODProp3D.h>

#include<stack>
#include<itkImageToVTKImageFilter.h>
//============================
#include<vtkFeatureEdges.h>

using internalImageType = itk::Image<float, 2>;
using internalImageType3D = itk::Image<float, 3>;
using readerType = itk::ImageFileReader<internalImageType>;
using readerType3D = itk::ImageSeriesReader<internalImageType3D>;
using NamesGeneratorType = itk::GDCMSeriesFileNames;
using NamesGeneratorType1 = itk::NumericSeriesFileNames;
using GDCMType = itk::GDCMImageIO;
using binaryThresholdFilterType = itk::BinaryThresholdImageFilter<internalImageType3D, internalImageType3D>;
using curvatureFlowImageFilter = itk::CurvatureFlowImageFilter<internalImageType, internalImageType>;
using connectedThresholdFilterType = itk::ConnectedThresholdImageFilter<internalImageType3D, internalImageType3D>;

using extractImageFilterType = itk::ExtractImageFilter<internalImageType3D, internalImageType>;
using joinImageFilterType = itk::JoinImageFilter<internalImageType, internalImageType3D>;
using joinSeriesImageFilterType = itk::JoinSeriesImageFilter<internalImageType, internalImageType3D>;
using neighborhoodIteratorType = itk::NeighborhoodIterator<internalImageType>;
using imageRegionIteratorType = itk::ImageRegionIterator<internalImageType>;
using linearIteratorType = itk::ImageLinearIteratorWithIndex<internalImageType>;
using slicerIteratorType = itk::ImageSliceIteratorWithIndex<internalImageType3D>;
using sliceBySliceFilterType = itk::SliceBySliceImageFilter<internalImageType3D, internalImageType3D>;

using itkToVTKimageFilterType = itk::ImageToVTKImageFilter<internalImageType3D>;

using writerType3D = itk::ImageSeriesWriter<internalImageType3D, internalImageType>;
using hausdorffDistanceFilterType = itk::HausdorffDistanceImageFilter<internalImageType, internalImageType>;


class LineInteractorStyle : public vtkInteractorStyleImage
{
public:
	static LineInteractorStyle *New();
	vtkTypeMacro(LineInteractorStyle, vtkInteractorStyleImage);

	~LineInteractorStyle();
	virtual void OnLeftButtonDown();

	internalImageType::Pointer itkImage = nullptr;
private:

};

vtkStandardNewMacro(LineInteractorStyle);

LineInteractorStyle::~LineInteractorStyle()
{
}

void LineInteractorStyle::OnLeftButtonDown()
{
	if (this->Interactor->GetControlKey())//this指向LinInteractorStyle
	{
		//视口坐标获取
		int *clickPos;
		clickPos = this->GetInteractor()->GetEventPosition();
		double point1[3];
		point1[0] = clickPos[0];
		point1[1] = clickPos[1];

		//世界坐标变换
		double worldPoint[3];
		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->SetDisplayPoint(point1[0], point1[1], point1[2]);
		this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->DisplayToWorld();
		worldPoint[0] = (this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetWorldPoint())[0];
		worldPoint[1] = (this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetWorldPoint())[1];
		worldPoint[2] = (this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetWorldPoint())[2];

		using PointType = itk::Point<double, 2>;
		PointType point;
		point[0] = worldPoint[0];
		point[1] = worldPoint[1];

		internalImageType::IndexType pixelIndex;
		bool isInside = this->itkImage->TransformPhysicalPointToIndex(point, pixelIndex);

		if (!isInside)
			return;
		/*cout << clickPos[0] << "  " << clickPos[1] << "  " << clickPos[2] << endl;
		cout << point[0] << "  " << point[1] << "  " << point[2] << endl;*/
		cout << pixelIndex[0] << "  " << pixelIndex[1] << "  " << pixelIndex[2] << endl;
		int range[2] = { 226,3071 };
		internalImageType::PixelType value = this->itkImage->GetPixel(pixelIndex);
		cout << value << endl;
		/*if (value<range[0] || value>range[1])
			return;*/
			/*internalImageType::Pointer image1 = ConnectedThersholdFilter(range, this->itkImage, pixelIndex);
			vtkSmartPointer<vtkImageData> image2 = itkTovtk(image1);
			vtkSmartPointer<vtkLookupTable> ColorTable = vtkSmartPointer<vtkLookupTable>::New();
			ColorTable->SetNumberOfColors(2);
			ColorTable->SetTableRange(0, 3071);
			ColorTable->SetTableValue(0, 0, 0, 0, 0);
			static bool flag = true;
			if (flag)
				ColorTable->SetTableValue(1, 0, 1, 0, 1);
			else
				ColorTable->SetTableValue(1, 0, 0, 0, 1);
			flag = !flag;
			ColorTable->Build();

			vtkSmartPointer<vtkImageMapToColors> ColorMap = vtkSmartPointer<vtkImageMapToColors>::New();
			ColorMap->SetInputData(image2);
			ColorMap->SetLookupTable(ColorTable);
			ColorMap->Update();

			vtkSmartPointer<vtkImageActor> ColorActor = vtkSmartPointer<vtkImageActor>::New();
			ColorActor->SetInputData(ColorMap->GetOutput());
			ColorActor->SetInterpolate(false);
			viewer->GetRenderer()->AddActor(ColorActor);
			viewer->Render();*/
	}
	else
	{
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}
}


class vtkSliderCallback :public vtkCommand
{
public:
	static vtkSliderCallback *New()
	{
		return new vtkSliderCallback;
	}
	virtual void Execute(vtkObject *caller, unsigned long, void*)
	{
		vtkSliderWidget *sliderWidget = reinterpret_cast<vtkSliderWidget*>(caller);
		this->Viewer->SetSlice(static_cast<vtkSliderRepresentation *>(sliderWidget->GetRepresentation())->GetValue());
		this->Viewer->Render();
	}
	vtkSliderCallback() {}
	vtkSmartPointer<vtkImageViewer2>Viewer = nullptr;
};


void indexConnected(internalImageType::IndexType index1, internalImageType::IndexType index2, internalImageType::Pointer outputimage)
{
	internalImageType::IndexType indexW;
	indexW = index1;
	//cout << indexW << endl << index2 << endl;
	outputimage->SetPixel(indexW, 8);
	int i = 0;
	while (indexW != index2)
	{
		int chose = 0;
		if ((indexW[0] < index2[0]) && (indexW[1] < index2[1]))
			chose = 1;
		if ((indexW[0] < index2[0]) && (indexW[1] == index2[1]))
			chose = 2;
		if ((indexW[0] < index2[0]) && (indexW[1] > index2[1]))
			chose = 3;
		if ((indexW[0] == index2[0]) && (indexW[1] < index2[1]))
			chose = 4;
		if ((indexW[0] == index2[0]) && (indexW[1] == index2[1]))
			chose = 5;
		if ((indexW[0] == index2[0]) && (indexW[1] > index2[1]))
			chose = 6;
		if ((indexW[0] > index2[0]) && (indexW[1] < index2[1]))
			chose = 7;
		if ((indexW[0] > index2[0]) && (indexW[1] == index2[1]))
			chose = 8;
		if ((indexW[0] > index2[0]) && (indexW[1] > index2[1]))
			chose = 9;

		switch (chose)
		{
		case 1:
		{
			//cout << "a" << endl;
			indexW[0]++;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]--;
				indexW[1]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1]--;
					outputimage->SetPixel(indexW, 8);
					break;
				}
			}
			//cout << indexW << endl;
			break;
		}
		case 2:
		{
			//cout << "b" << endl;
			indexW[0]++;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]--;
				indexW[1]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1] = indexW[1] - 2;
					if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
					{
						//outputimage7->SetPixel(indexW, 10);
						outputimage->SetPixel(indexW, 8);
					}
					else
					{
						outputimage->SetPixel(indexW, 8);
						break;
					}
				}
			}
			break;
		}
		case 3:
		{
			//cout << "c" << endl;
			indexW[0]++;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]--;
				indexW[1]--;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1]++;
					outputimage->SetPixel(indexW, 8);
					break;
				}
			}
			break;
		}
		case 4:
		{
			//cout << "d" << endl;
			indexW[1]++;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[1]--;
				indexW[0]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[0] = indexW[0] - 2;
					if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
					{
						//outputimage7->SetPixel(indexW, 10);
						outputimage->SetPixel(indexW, 8);
					}
					else
					{
						outputimage->SetPixel(indexW, 8);
						break;
					}
				}
			}
			//cout << indexW << endl;
			break;
		}
		case 5:
		{
			//cout << "e" << endl;
			outputimage->SetPixel(indexW, 8);
			break;
		}
		case 6:
		{
			//cout << "f" << endl;
			indexW[1]--;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[1]++;
				indexW[0]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[0] = indexW[0] - 2;
					if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
					{
						//outputimage7->SetPixel(indexW, 10);
						outputimage->SetPixel(indexW, 8);
					}
					else
					{
						outputimage->SetPixel(indexW, 8);
						break;
					}
				}
			}
			break;
		}
		case 7:
		{
			//cout << "g" << endl;
			indexW[0]--;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]++;
				indexW[1]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1]--;
					outputimage->SetPixel(indexW, 8);
					break;
				}
			}
			//cout << indexW << endl;
			break;
		}
		case 8:
		{
			//cout << "h" << endl;
			indexW[0]--;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]++;
				indexW[1]++;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1] = indexW[1] - 2;
					if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
					{
						//outputimage7->SetPixel(indexW, 10);
						outputimage->SetPixel(indexW, 8);
					}
					else
					{
						outputimage->SetPixel(indexW, 8);
						break;
					}
				}
			}
			break;
		}
		case 9:
		{
			//cout << "i" << endl;
			indexW[0]--;
			if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
			{
				//outputimage7->SetPixel(indexW, 10);
				outputimage->SetPixel(indexW, 8);

			}
			else
			{
				indexW[0]++;
				indexW[1]--;
				if ((outputimage->GetPixel(indexW) == 7) || (outputimage->GetPixel(indexW) == 10))
				{
					//outputimage7->SetPixel(indexW, 10);
					outputimage->SetPixel(indexW, 8);
				}
				else
				{
					indexW[1]++;
					outputimage->SetPixel(indexW, 8);
					break;
				}
			}
			break;
		}
		default:
			break;
		}
		i++;
		if (i > 10)
		{
			//cout << "路径过长" << endl;
			break;
		}
	}
	//cout << "done" << endl;
}


//void floodFill(internalImageType3D::IndexType pixelIndex, float oldValue, float newValue, internalImageType3D::Pointer inputImage)
//{
//	internalImageType::PixelType value = inputImage->GetPixel(pixelIndex);
//	if (value == oldValue)
//	{
//		inputImage->SetPixel(pixelIndex, newValue);
//		internalImageType3D::IndexType pixelIndex1, pixelIndex2, pixelIndex3, pixelIndex4, pixelIndex5, pixelIndex6;
//		/*internalImageType3D::IndexType* pixelIndex1 = new internalImageType3D::IndexType;
//		internalImageType3D::IndexType* pixelIndex2 = new internalImageType3D::IndexType;
//		internalImageType3D::IndexType* pixelIndex3 = new internalImageType3D::IndexType;
//		internalImageType3D::IndexType* pixelIndex4 = new internalImageType3D::IndexType;
//		internalImageType3D::IndexType* pixelIndex5 = new internalImageType3D::IndexType;
//		internalImageType3D::IndexType* pixelIndex6 = new internalImageType3D::IndexType;*/
//
//
//		pixelIndex1[0] = pixelIndex[0] + 1;
//		pixelIndex1[1] = pixelIndex[1];
//		pixelIndex1[2] = pixelIndex[2];
//
//		pixelIndex2[0] = pixelIndex[0];
//		pixelIndex2[1] = pixelIndex[1] + 1;
//		pixelIndex2[2] = pixelIndex[2];
//
//		pixelIndex3[0] = pixelIndex[0] - 1;
//		pixelIndex3[1] = pixelIndex[1];
//		pixelIndex3[2] = pixelIndex[2];
//
//		pixelIndex4[0] = pixelIndex[0];
//		pixelIndex4[1] = pixelIndex[1] - 1;
//		pixelIndex4[2] = pixelIndex[2];
//
//		pixelIndex5[0] = pixelIndex[0];
//		pixelIndex5[1] = pixelIndex[1];
//		pixelIndex5[2] = pixelIndex[2] + 1;
//
//		pixelIndex6[0] = pixelIndex[0];
//		pixelIndex6[1] = pixelIndex[1];
//		pixelIndex6[2] = pixelIndex[2] - 1;
//
//		floodFill(pixelIndex1, oldValue, newValue, inputImage);
//		floodFill(pixelIndex2, oldValue, newValue, inputImage);
//		floodFill(pixelIndex3, oldValue, newValue, inputImage);
//		floodFill(pixelIndex4, oldValue, newValue, inputImage);
//		floodFill(pixelIndex5, oldValue, newValue, inputImage);
//		floodFill(pixelIndex6, oldValue, newValue, inputImage);
//	}
//	else
//		return;
//}


class FloodFillNIMABI
{
private:
	std::stack<internalImageType3D::IndexType>pixelIndexS1;
	std::stack<internalImageType::IndexType>pixelIndexS2;
	std::stack<internalImageType::IndexType>pixelIndexS3;
	std::stack<internalImageType::IndexType>pixelIndexS4;


public:
	bool checkNewSeed(internalImageType3D::IndexType pixelIndex, float oldValue, float newValue, internalImageType3D::Pointer inputImage);
	void FillIterator(internalImageType3D::IndexType pixelIndex, float oldValue, float newValue, internalImageType3D::Pointer inputImage);
};

bool FloodFillNIMABI::checkNewSeed(internalImageType3D::IndexType pixelIndex, float oldValue, float newValue, internalImageType3D::Pointer inputImage)
{
	internalImageType3D::IndexType pixelIndex1, pixelIndex2, pixelIndex3, pixelIndex4;
	internalImageType3D::PixelType value = inputImage->GetPixel(pixelIndex);
	if (value == oldValue)
	{
		inputImage->SetPixel(pixelIndex, newValue);
		/*pixelIndex1[0] = pixelIndex[0] + 1;
		pixelIndex1[1] = pixelIndex[1];
		pixelIndex1[2] = pixelIndex[2];

		pixelIndex2[0] = pixelIndex[0];
		pixelIndex2[1] = pixelIndex[1] + 1;
		pixelIndex2[2] = pixelIndex[2];

		pixelIndex3[0] = pixelIndex[0] - 1;
		pixelIndex3[1] = pixelIndex[1];
		pixelIndex3[2] = pixelIndex[2];

		pixelIndex4[0] = pixelIndex[0];
		pixelIndex4[1] = pixelIndex[1] - 1;
		pixelIndex4[2] = pixelIndex[2];

		pixelIndexS1.push(pixelIndex1);
		pixelIndexS2.push(pixelIndex2);
		pixelIndexS3.push(pixelIndex3);
		pixelIndexS4.push(pixelIndex4);*/
		pixelIndexS1.push(pixelIndex);

		return true;
	}

	return false;
}


void FloodFillNIMABI::FillIterator(internalImageType3D::IndexType pixelIndex, float oldValue, float newValue, internalImageType3D::Pointer inputImage)
{
	internalImageType3D::IndexType pixelIndexR, pixelIndexRu, pixelIndexRd, pixelIndexU, pixelIndexL, pixelIndexLu, pixelIndexLd, pixelIndexD, pixelIndexF, pixelIndexB;
	//checkNewSeed(pixelIndex, oldValue, newValue, inputImage);
	pixelIndexS1.push(pixelIndex);

	while (true)
	{
		/*if (!pixelIndexS1.empty())
		{
			pixelIndexR = pixelIndexS1.top();
			pixelIndexS1.pop();
			checkNewSeed(pixelIndexR, oldValue, newValue, inputImage);
		}
		if (!pixelIndexS2.empty())
		{
			pixelIndexU = pixelIndexS2.top();
			pixelIndexS2.pop();
			checkNewSeed(pixelIndexU, oldValue, newValue, inputImage);
		}
		if (!pixelIndexS3.empty())
		{
			pixelIndexL = pixelIndexS3.top();
			pixelIndexS3.pop();
			checkNewSeed(pixelIndexL, oldValue, newValue, inputImage);
		}
		if (!pixelIndexS4.empty())
		{
			pixelIndexD = pixelIndexS4.top();
			pixelIndexS4.pop();
			checkNewSeed(pixelIndexD, oldValue, newValue, inputImage);

		}*/
		if (!pixelIndexS1.empty())
		{
			pixelIndex = pixelIndexS1.top();
			pixelIndexS1.pop();
			pixelIndexR[0] = pixelIndex[0] + 1;
			pixelIndexR[1] = pixelIndex[1];
			pixelIndexR[2] = pixelIndex[2];

			/*pixelIndexRu[0] = pixelIndex[0] + 1;
			pixelIndexRu[1] = pixelIndex[1] + 1;
			pixelIndexRu[2] = pixelIndex[2];*/

			/*pixelIndexRd[0] = pixelIndex[0] + 1;
			pixelIndexRd[1] = pixelIndex[1] - 1;
			pixelIndexRd[2] = pixelIndex[2];*/

			pixelIndexU[0] = pixelIndex[0];
			pixelIndexU[1] = pixelIndex[1] + 1;
			pixelIndexU[2] = pixelIndex[2];

			pixelIndexL[0] = pixelIndex[0] - 1;
			pixelIndexL[1] = pixelIndex[1];
			pixelIndexL[2] = pixelIndex[2];

			/*pixelIndexLu[0] = pixelIndex[0] - 1;
			pixelIndexLu[1] = pixelIndex[1] + 1;
			pixelIndexLu[2] = pixelIndex[2];*/

			/*pixelIndexLd[0] = pixelIndex[0] - 1;
			pixelIndexLd[1] = pixelIndex[1] - 1;
			pixelIndexLd[2] = pixelIndex[2];*/

			pixelIndexD[0] = pixelIndex[0];
			pixelIndexD[1] = pixelIndex[1] - 1;
			pixelIndexD[2] = pixelIndex[2];

			pixelIndexF[0] = pixelIndex[0];
			pixelIndexF[1] = pixelIndex[1];
			pixelIndexF[2] = pixelIndex[2] + 1;

			pixelIndexB[0] = pixelIndex[0];
			pixelIndexB[1] = pixelIndex[1];
			pixelIndexB[2] = pixelIndex[2] - 1;

			checkNewSeed(pixelIndex, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexR, oldValue, newValue, inputImage);
			//checkNewSeed(pixelIndexRu, oldValue, newValue, inputImage);
			//checkNewSeed(pixelIndexRd, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexU, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexL, oldValue, newValue, inputImage);
			//checkNewSeed(pixelIndexLu, oldValue, newValue, inputImage);
			//checkNewSeed(pixelIndexLd, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexD, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexF, oldValue, newValue, inputImage);
			checkNewSeed(pixelIndexB, oldValue, newValue, inputImage);

		}
		else
			break;

	}

}




int main()
{
	//==============================SE0导入===========================
	GDCMType::Pointer DicomIO = GDCMType::New();
	NamesGeneratorType::Pointer NameGenerator = NamesGeneratorType::New();

	readerType3D::Pointer reader = readerType3D::New();
	reader->SetImageIO(DicomIO);
	NameGenerator->SetUseSeriesDetails(true);
	NameGenerator->AddSeriesRestriction("0008|0021");
	NameGenerator->SetDirectory("F:\\SE0");

	using SeriesIDContainer = std::vector<std::string>;
	const SeriesIDContainer & seriesUID = NameGenerator->GetSeriesUIDs();
	auto seriesItr = seriesUID.begin();
	auto seriesEnd = seriesUID.end();
	using FileNameContainer = std::vector<std::string>;
	FileNameContainer FileNames;
	std::string seriesIdentifier;
	while (seriesItr != seriesEnd)
	{
		seriesIdentifier = seriesItr->c_str();
		std::cout << seriesItr->c_str() << std::endl;
		FileNames = NameGenerator->GetFileNames(seriesIdentifier);
		++seriesItr;
	}

	reader->SetFileNames(FileNames);

	try
	{
		reader->Update();
	}
	catch (const itk::ExceptionObject & ex)
	{
		std::cout << ex << std::endl;

	}
	internalImageType3D::SizeType imageSize = reader->GetOutput()->GetLargestPossibleRegion().GetSize();
	internalImageType3D::SpacingType imageSpacing = reader->GetOutput()->GetSpacing();
	std::cout << "read done！Original size: " << imageSize << std::endl;

	//================================2d
	readerType::Pointer reader2d = readerType::New();
	reader2d->SetFileName("F:\\SE0\\IM527.dcm");
	reader2d->SetImageIO(DicomIO);
	reader2d->Update();

	//====================================SE1导入=========================
	GDCMType::Pointer DicomIO1 = GDCMType::New();
	NamesGeneratorType::Pointer NameGenerator1 = NamesGeneratorType::New();
	readerType3D::Pointer reader1 = readerType3D::New();
	reader1->SetImageIO(DicomIO1);
	NameGenerator1->SetUseSeriesDetails(true);
	NameGenerator1->AddSeriesRestriction("0008|0021");
	NameGenerator1->SetDirectory("F:\\SE6");

	const SeriesIDContainer & seriesUID1 = NameGenerator1->GetSeriesUIDs();
	auto seriesItr1 = seriesUID1.begin();
	auto seriesEnd1 = seriesUID1.end();
	FileNameContainer FileNames1;
	std::string seriesIdentifier1;
	while (seriesItr1 != seriesEnd1)
	{
		seriesIdentifier1 = seriesItr1->c_str();
		std::cout << seriesItr1->c_str() << std::endl;
		FileNames1 = NameGenerator1->GetFileNames(seriesIdentifier1);
		++seriesItr1;
	}

	reader1->SetFileNames(FileNames1);

	try
	{
		reader1->Update();
	}
	catch (const itk::ExceptionObject & ex)
	{
		std::cout << ex << std::endl;

	}
	internalImageType3D::SizeType imageSize1 = reader1->GetOutput()->GetLargestPossibleRegion().GetSize();
	//internalImageType3D::SpacingType imageSpacing1 = reader->GetOutput()->GetSpacing();
	std::cout << "reader1 done！Original size: " << imageSize1 << std::endl;

	//====================================观测SE4导入=========================
	NamesGeneratorType::Pointer NameGenerator2 = NamesGeneratorType::New();
	readerType3D::Pointer reader2 = readerType3D::New();
	reader2->SetImageIO(DicomIO);
	NameGenerator2->SetUseSeriesDetails(true);
	NameGenerator2->AddSeriesRestriction("0008|0021");
	NameGenerator2->SetDirectory("F:\\SE4");

	const SeriesIDContainer & seriesUID2 = NameGenerator2->GetSeriesUIDs();
	auto seriesItr2 = seriesUID2.begin();
	auto seriesEnd2 = seriesUID2.end();
	FileNameContainer FileNames2;
	std::string seriesIdentifier2;
	while (seriesItr2 != seriesEnd2)
	{
		seriesIdentifier2 = seriesItr2->c_str();
		std::cout << seriesItr2->c_str() << std::endl;
		FileNames2 = NameGenerator2->GetFileNames(seriesIdentifier2);
		++seriesItr2;
	}

	reader2->SetFileNames(FileNames2);

	try
	{
		reader2->Update();
	}
	catch (const itk::ExceptionObject & ex)
	{
		std::cout << ex << std::endl;

	}
	internalImageType3D::SizeType imageSize2 = reader2->GetOutput()->GetLargestPossibleRegion().GetSize();
	std::cout << "reader2 done！Original size: " << imageSize2 << std::endl;

	//============================区域生长分割二值化=====================================
   /* binaryThresholdFilterType::Pointer binaryFilter1 = binaryThresholdFilterType::New();
	binaryFilter1->SetInput(reader->GetOutput());
	binaryFilter1->SetLowerThreshold(186);
	binaryFilter1->SetUpperThreshold(3071);
	binaryFilter1->SetInsideValue(1);
	binaryFilter1->SetOutsideValue(0);
	binaryFilter1->Update();*/

	/* internalImageType3D::IndexType seed1,seed2;
	 seed1[0] = 357;
	 seed1[1] = 255;
	 seed1[2] = 253;
	 seed2[0] = 342;
	 seed2[1] = 247;
	 seed2[2] = 253;*/

	 /* internalImageType3D::IndexType seed1, seed2, seed3, seed4;
	  seed1[0] = 350;
	  seed1[1] = 266;
	  seed1[2] = 421;
	  seed2[0] = 329;
	  seed2[1] = 248;
	  seed2[2] = 363;
	  seed3[0] = 357;
	  seed3[1] = 278;
	  seed3[2] = 421;
	  seed2[0] = 352;
	  seed2[1] = 271;
	  seed2[2] = 363;*/

	internalImageType3D::IndexType seed1, seed2;
	seed1[0] = 404;
	seed1[1] = 295;
	seed1[2] = 458;
	seed2[0] = 416;
	seed2[1] = 276;
	seed2[2] = 458;

	connectedThresholdFilterType::Pointer connectedThresholdFilter = connectedThresholdFilterType::New();
	connectedThresholdFilter->SetInput(reader->GetOutput());
	connectedThresholdFilter->SetLower(226);
	connectedThresholdFilter->SetUpper(3071);
	connectedThresholdFilter->AddSeed(seed1);
	connectedThresholdFilter->AddSeed(seed2);
	connectedThresholdFilter->SetReplaceValue(1);
	connectedThresholdFilter->Update();

	extractImageFilterType::Pointer extractFilter = extractImageFilterType::New();
	//joinImageFilterType::Pointer joinFilter = joinImageFilterType::New();
	joinSeriesImageFilterType::Pointer joinFilter = joinSeriesImageFilterType::New();
	extractFilter->InPlaceOn();
	extractFilter->SetDirectionCollapseToIdentity();
	extractFilter->SetInput(connectedThresholdFilter->GetOutput());
	internalImageType3D::RegionType extractRegion = reader->GetOutput()->GetLargestPossibleRegion();
	internalImageType3D::SizeType size = extractRegion.GetSize();

	internalImageType3D::IndexType index = extractRegion.GetIndex();

	joinFilter->SetOrigin(0);
	joinFilter->SetSpacing(imageSpacing[2]);

	for (int i = 0; i < imageSize[2]; i++)
	{
		index[2] = i;
		size[2] = 0;
		extractRegion.SetSize(size);
		extractRegion.SetIndex(index);
		extractFilter->SetExtractionRegion(extractRegion);
		extractFilter->Update();

		internalImageType::Pointer inputimage = internalImageType::New();
		inputimage = extractFilter->GetOutput();
		internalImageType::Pointer outputimage = internalImageType::New();
		outputimage->SetRegions(extractFilter->GetOutput()->GetRequestedRegion());
		outputimage->CopyInformation(extractFilter->GetOutput());
		outputimage->Allocate();

		neighborhoodIteratorType::RadiusType radius;
		radius.Fill(1);
		neighborhoodIteratorType::RadiusType radius1;
		radius1.Fill(3);
		neighborhoodIteratorType::RadiusType radius2;
		radius2.Fill(4);

		neighborhoodIteratorType::OffsetType offset1 = { {1,0} };
		neighborhoodIteratorType::OffsetType offset2 = { {1,-1} };
		neighborhoodIteratorType::OffsetType offset3 = { {0,-1} };
		neighborhoodIteratorType::OffsetType offset4 = { {-1,-1} };
		neighborhoodIteratorType::OffsetType offset5 = { {-1,0} };
		neighborhoodIteratorType::OffsetType offset6 = { {-1,1} };
		neighborhoodIteratorType::OffsetType offset7 = { {0,1} };
		neighborhoodIteratorType::OffsetType offset8 = { {1,1} };
		neighborhoodIteratorType::OffsetType offset9 = { {2,0} };
		neighborhoodIteratorType::OffsetType offset10 = { {2,-1} };
		neighborhoodIteratorType::OffsetType offset11 = { {2,-2} };
		neighborhoodIteratorType::OffsetType offset12 = { {1,-2} };
		neighborhoodIteratorType::OffsetType offset13 = { {0,-2} };
		neighborhoodIteratorType::OffsetType offset14 = { {-1,-2} };
		neighborhoodIteratorType::OffsetType offset15 = { {-2,-2} };
		neighborhoodIteratorType::OffsetType offset16 = { {-2,-1} };
		neighborhoodIteratorType::OffsetType offset17 = { {-2,0} };
		neighborhoodIteratorType::OffsetType offset18 = { {-2,1} };
		neighborhoodIteratorType::OffsetType offset19 = { {-2,2} };
		neighborhoodIteratorType::OffsetType offset20 = { {-1,2} };
		neighborhoodIteratorType::OffsetType offset21 = { {0,2} };
		neighborhoodIteratorType::OffsetType offset22 = { {1,2} };
		neighborhoodIteratorType::OffsetType offset23 = { {2,2} };
		neighborhoodIteratorType::OffsetType offset24 = { {2,1} };
		neighborhoodIteratorType::OffsetType offset25 = { {3,0} };
		neighborhoodIteratorType::OffsetType offset26 = { {3,-1} };
		neighborhoodIteratorType::OffsetType offset27 = { {3,-2} };
		neighborhoodIteratorType::OffsetType offset28 = { {3,-3} };
		neighborhoodIteratorType::OffsetType offset29 = { {2,-3} };
		neighborhoodIteratorType::OffsetType offset30 = { {1,-3} };
		neighborhoodIteratorType::OffsetType offset31 = { {0,-3} };
		neighborhoodIteratorType::OffsetType offset32 = { {-1,-3} };
		neighborhoodIteratorType::OffsetType offset33 = { {-2,-3} };
		neighborhoodIteratorType::OffsetType offset34 = { {-3,-3} };
		neighborhoodIteratorType::OffsetType offset35 = { {-3,-2} };
		neighborhoodIteratorType::OffsetType offset36 = { {-3,-1} };
		neighborhoodIteratorType::OffsetType offset37 = { {-3,0} };
		neighborhoodIteratorType::OffsetType offset38 = { {-3,1} };
		neighborhoodIteratorType::OffsetType offset39 = { {-3,2} };
		neighborhoodIteratorType::OffsetType offset40 = { {-3,3} };
		neighborhoodIteratorType::OffsetType offset41 = { {-2,3} };
		neighborhoodIteratorType::OffsetType offset42 = { {-1,3} };
		neighborhoodIteratorType::OffsetType offset43 = { {0,3} };
		neighborhoodIteratorType::OffsetType offset44 = { {1,3} };
		neighborhoodIteratorType::OffsetType offset45 = { {2,3} };
		neighborhoodIteratorType::OffsetType offset46 = { {3,3} };
		neighborhoodIteratorType::OffsetType offset47 = { {3,2} };
		neighborhoodIteratorType::OffsetType offset48 = { {3,1} };
		neighborhoodIteratorType::OffsetType offset49 = { {4,0} };
		neighborhoodIteratorType::OffsetType offset50 = { {4,-1} };
		neighborhoodIteratorType::OffsetType offset51 = { {4,-2} };
		neighborhoodIteratorType::OffsetType offset52 = { {4,-3} };
		neighborhoodIteratorType::OffsetType offset53 = { {4,-4} };
		neighborhoodIteratorType::OffsetType offset54 = { {3,-4} };
		neighborhoodIteratorType::OffsetType offset55 = { {2,-4} };
		neighborhoodIteratorType::OffsetType offset56 = { {1,-4} };
		neighborhoodIteratorType::OffsetType offset57 = { {0,-4} };
		neighborhoodIteratorType::OffsetType offset58 = { {-1,-4} };
		neighborhoodIteratorType::OffsetType offset59 = { {-2,-4} };
		neighborhoodIteratorType::OffsetType offset60 = { {-3,-4} };
		neighborhoodIteratorType::OffsetType offset61 = { {-4,-4} };
		neighborhoodIteratorType::OffsetType offset62 = { {-4,-3} };
		neighborhoodIteratorType::OffsetType offset63 = { {-4,-2} };
		neighborhoodIteratorType::OffsetType offset64 = { {-4,-1} };
		neighborhoodIteratorType::OffsetType offset65 = { {-4,0} };
		neighborhoodIteratorType::OffsetType offset66 = { {-4,1} };
		neighborhoodIteratorType::OffsetType offset67 = { {-4,2} };
		neighborhoodIteratorType::OffsetType offset68 = { {-4,3} };
		neighborhoodIteratorType::OffsetType offset69 = { {-4,4} };
		neighborhoodIteratorType::OffsetType offset70 = { {-3,4} };
		neighborhoodIteratorType::OffsetType offset71 = { {-2,4} };
		neighborhoodIteratorType::OffsetType offset72 = { {-1,4} };
		neighborhoodIteratorType::OffsetType offset73 = { {0,4} };
		neighborhoodIteratorType::OffsetType offset74 = { {1,4} };
		neighborhoodIteratorType::OffsetType offset75 = { {2,4} };
		neighborhoodIteratorType::OffsetType offset76 = { {3,4} };
		neighborhoodIteratorType::OffsetType offset77 = { {4,4} };
		neighborhoodIteratorType::OffsetType offset78 = { {4,3} };
		neighborhoodIteratorType::OffsetType offset79 = { {4,2} };
		neighborhoodIteratorType::OffsetType offset80 = { {4,1} };

		//=================================补洞=====================================

		/*neighborhoodIteratorType inputit8(radius, inputimage, inputimage->GetRequestedRegion());
		imageRegionIteratorType outputit8(outputimage8, outputimage8->GetRequestedRegion());
		internalImageType::IndexType indexH[10];
		int h = 0;
		for (inputit8.GoToBegin(), outputit8.GoToBegin(); !inputit8.IsAtEnd(); ++inputit8, ++outputit8)
		{
			if (inputit8.GetCenterPixel() == 0)
			{
				if (inputit8.GetPixel(offset1) != 0 &&
					inputit8.GetPixel(offset3) != 0 &&
					inputit8.GetPixel(offset5) != 0 &&
					inputit8.GetPixel(offset7) != 0)
				{
					outputit8.Set(1);
					cout << inputit8.GetIndex() << endl;
					indexH[h] = inputit8.GetIndex();
					h++;
				}
				else
				{
					outputit8.Set(0);
				}

			}
			else
			{
				outputit8.Set(inputit8.GetCenterPixel());
			}
		}*/

		//=================================4-邻域======================================
		neighborhoodIteratorType inputit(radius, inputimage, inputimage->GetRequestedRegion());
		imageRegionIteratorType outputit(outputimage, outputimage->GetRequestedRegion());

		for (inputit.GoToBegin(), outputit.GoToBegin(); !inputit.IsAtEnd(); ++inputit, ++outputit)
		{
			int s = 0;
			if (inputit.GetCenterPixel() == 1)
			{
				s = inputit.GetPixel(offset1) - inputit.GetPixel(offset1)*inputit.GetPixel(offset2)*inputit.GetPixel(offset3);
				s += inputit.GetPixel(offset3) - inputit.GetPixel(offset3)*inputit.GetPixel(offset4)*inputit.GetPixel(offset5);
				s += inputit.GetPixel(offset5) - inputit.GetPixel(offset5)*inputit.GetPixel(offset6)*inputit.GetPixel(offset7);
				s += inputit.GetPixel(offset7) - inputit.GetPixel(offset7)*inputit.GetPixel(offset8)*inputit.GetPixel(offset1);

				if (inputit.GetPixel(offset1) == 0 &&
					inputit.GetPixel(offset2) == 0 &&
					inputit.GetPixel(offset3) == 0 &&
					inputit.GetPixel(offset4) == 0 &&
					inputit.GetPixel(offset5) == 0 &&
					inputit.GetPixel(offset6) == 0 &&
					inputit.GetPixel(offset7) == 0 &&
					inputit.GetPixel(offset8) == 0)
				{
					outputit.Set(6);
				}
				else
				{
					switch (s)
					{
					case 0:
						outputit.Set(8);
						break;
					case 1:
						outputit.Set(2);
						break;
					case 2:
						outputit.Set(3);
						break;
					case 3:
						outputit.Set(4);
						break;
					case 4:
						outputit.Set(5);
						break;
					default:
						cout << "erro calculate!" << endl;
					}
				}
			}
			/*else
			{
				outputit.Set(0);
			}*/
		}

		//=================================4-邻域-边界点检测===========================
		internalImageType::Pointer inputimage1 = internalImageType::New();
		inputimage1 = outputimage;
		internalImageType::Pointer outputimage1 = internalImageType::New();
		outputimage1->SetRegions(inputimage1->GetRequestedRegion());
		outputimage1->CopyInformation(inputimage1);
		outputimage1->Allocate();

		neighborhoodIteratorType inputit1(radius, inputimage1, inputimage1->GetRequestedRegion());
		imageRegionIteratorType outputit1(outputimage1, outputimage1->GetRequestedRegion());

		for (inputit1.GoToBegin(), outputit1.GoToBegin(); !inputit1.IsAtEnd(); ++inputit1, ++outputit1)
		{
			int i = 0;
			if (inputit1.GetCenterPixel() == 8)
			{
				if (inputit1.GetPixel(offset1) == 2)
					i++;
				if (inputit1.GetPixel(offset2) == 2)
					i++;
				if (inputit1.GetPixel(offset3) == 2)
					i++;
				if (inputit1.GetPixel(offset4) == 2)
					i++;
				if (inputit1.GetPixel(offset5) == 2)
					i++;
				if (inputit1.GetPixel(offset6) == 2)
					i++;
				if (inputit1.GetPixel(offset7) == 2)
					i++;
				if (inputit1.GetPixel(offset8) == 2)
					i++;
				if (i < 3)
				{
					outputit1.Set(7);
				}
				else
				{
					outputit1.Set(8);
				}

			}

			else
			{
				outputit1.Set(inputit1.GetCenterPixel());
			}

		}

		//================================凹点检测===============================
		internalImageType::Pointer outputimage5 = internalImageType::New();
		outputimage5->SetRegions(outputimage1->GetRequestedRegion());
		outputimage5->CopyInformation(outputimage1);
		outputimage5->Allocate();

		neighborhoodIteratorType inputit6(radius2, outputimage1, outputimage1->GetRequestedRegion());
		imageRegionIteratorType outputit6(outputimage5, outputimage5->GetRequestedRegion());

		for (inputit6.GoToBegin(), outputit6.GoToBegin(); !inputit6.IsAtEnd(); ++inputit6, ++outputit6)
		{
			int s = 0;
			if (inputit6.GetCenterPixel() == 8)
			{
				if (inputit6.GetPixel(offset1) == 0)
					s++;
				if (inputit6.GetPixel(offset2) == 0)
					s++;
				if (inputit6.GetPixel(offset3) == 0)
					s++;
				if (inputit6.GetPixel(offset4) == 0)
					s++;
				if (inputit6.GetPixel(offset5) == 0)
					s++;
				if (inputit6.GetPixel(offset6) == 0)
					s++;
				if (inputit6.GetPixel(offset7) == 0)
					s++;
				if (inputit6.GetPixel(offset8) == 0)
					s++;
				if (inputit6.GetPixel(offset9) == 0)
					s++;
				if (inputit6.GetPixel(offset10) == 0)
					s++;
				if (inputit6.GetPixel(offset11) == 0)
					s++;
				if (inputit6.GetPixel(offset12) == 0)
					s++;
				if (inputit6.GetPixel(offset13) == 0)
					s++;
				if (inputit6.GetPixel(offset14) == 0)
					s++;
				if (inputit6.GetPixel(offset15) == 0)
					s++;
				if (inputit6.GetPixel(offset16) == 0)
					s++;
				if (inputit6.GetPixel(offset17) == 0)
					s++;
				if (inputit6.GetPixel(offset18) == 0)
					s++;
				if (inputit6.GetPixel(offset19) == 0)
					s++;
				if (inputit6.GetPixel(offset20) == 0)
					s++;
				if (inputit6.GetPixel(offset21) == 0)
					s++;
				if (inputit6.GetPixel(offset22) == 0)
					s++;
				if (inputit6.GetPixel(offset23) == 0)
					s++;
				if (inputit6.GetPixel(offset24) == 0)
					s++;
				if (inputit6.GetPixel(offset25) == 0)
					s++;
				/*if (inputit6.GetPixel(offset26) == 0)
					s++;
				if (inputit6.GetPixel(offset27) == 0)
					s++;
				if (inputit6.GetPixel(offset28) == 0)
					s++;
				if (inputit6.GetPixel(offset29) == 0)
					s++;
				if (inputit6.GetPixel(offset30) == 0)
					s++;*/
				if (inputit6.GetPixel(offset31) == 0)
					s++;
				/*if (inputit6.GetPixel(offset32) == 0)
					s++;
				if (inputit6.GetPixel(offset33) == 0)
					s++;
				if (inputit6.GetPixel(offset34) == 0)
					s++;
				if (inputit6.GetPixel(offset35) == 0)
					s++;
				if (inputit6.GetPixel(offset36) == 0)
					s++;*/
				if (inputit6.GetPixel(offset37) == 0)
					s++;
				/*if (inputit6.GetPixel(offset38) == 0)
					s++;
				if (inputit6.GetPixel(offset39) == 0)
					s++;
				if (inputit6.GetPixel(offset40) == 0)
					s++;
				if (inputit6.GetPixel(offset41) == 0)
					s++;
				if (inputit6.GetPixel(offset42) == 0)
					s++;*/
				if (inputit6.GetPixel(offset43) == 0)
					s++;
				/*	if (inputit6.GetPixel(offset44) == 0)
						s++;
					if (inputit6.GetPixel(offset45) == 0)
						s++;
					if (inputit6.GetPixel(offset46) == 0)
						s++;
					if (inputit6.GetPixel(offset47) == 0)
						s++;
					if (inputit6.GetPixel(offset48) == 0)
						s++;
					if (inputit6.GetPixel(offset49) == 0)
						s++;*/

				if (s < 5 && s > 1)
				{
					outputit6.Set(10);
				}
				else
				{
					outputit6.Set(inputit6.GetCenterPixel());
				}
			}
			else
			{
				outputit6.Set(inputit6.GetCenterPixel());
			}
		}

		//============================白色点进一步划分===================


		internalImageType::Pointer outputimage7 = internalImageType::New();
		outputimage7->SetRegions(outputimage5->GetRequestedRegion());
		outputimage7->CopyInformation(outputimage5);
		outputimage7->Allocate();

		neighborhoodIteratorType inputit7(radius1, outputimage5, outputimage5->GetRequestedRegion());
		imageRegionIteratorType outputit7(outputimage7, outputimage7->GetRequestedRegion());

		int w = 0;
		internalImageType::IndexType index[50];
		int indexN = 0;

		for (inputit7.GoToBegin(), outputit7.GoToBegin(); !inputit7.IsAtEnd(); ++inputit7, ++outputit7)
		{
			int s = 0;
			int r = 0;

			if (inputit7.GetCenterPixel() == 10)
			{
				if ((inputit7.GetPixel(offset1) == 10) || (inputit7.GetPixel(offset1) == 9))
					s++;
				if ((inputit7.GetPixel(offset2) == 10) || (inputit7.GetPixel(offset2) == 9))
					s++;
				if ((inputit7.GetPixel(offset3) == 10) || (inputit7.GetPixel(offset3) == 9))
					s++;
				if ((inputit7.GetPixel(offset4) == 10) || (inputit7.GetPixel(offset4) == 9))
					s++;
				if ((inputit7.GetPixel(offset5) == 10) || (inputit7.GetPixel(offset5) == 9))
					s++;
				if ((inputit7.GetPixel(offset6) == 10) || (inputit7.GetPixel(offset6) == 9))
					s++;
				if ((inputit7.GetPixel(offset7) == 10) || (inputit7.GetPixel(offset7) == 9))
					s++;
				if ((inputit7.GetPixel(offset8) == 10) || (inputit7.GetPixel(offset8) == 9))
					s++;

				if (s > 0)
				{
					//outputit7.Set(11);
					outputit7.Set(8);
				}
				else
				{
					if (inputit7.GetPixel(offset9) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset10) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset11) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset12) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset13) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset14) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset15) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset16) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset17) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset18) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset19) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset20) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset21) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset22) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset23) == 10)
					{
						r++;
					}
					if (inputit7.GetPixel(offset24) == 10)
					{
						r++;
					}
					/*if (s > 0)
					{
						outputit7.Set(11);
					}
					else
					{*/
					if (r > 0)
					{
						outputit7.Set(inputit7.GetCenterPixel());
						//cout << "pixel1:" << r << endl;
						w++;
						index[indexN] = inputit7.GetIndex();

						//cout << w << endl;
						//cout << indexN << ":" << index[indexN] << endl;

						indexN++;
					}
					else
					{
						//outputit7.Set(11);
						outputit7.Set(8);
						//outputit7.Set(7);
						//cout << "pixel:" << r << endl;
						//cout << "index:" << s << endl;
					}
					/*}*/
				}


			}
			else
			{
				outputit7.Set(inputit7.GetCenterPixel());
			}

		}

		//=============================检测凹点索引、个数==================================

		if (w == 1)
		{
			outputimage7->SetPixel(index[0], 8);
		}
		else
		{
			for (int i = 0; i < w; i++)
			{
				float s = 999;
				int p = 0;
				internalImageType::IndexType indexE;
				indexE = index[i];
				for (int o = i + 1; o < w; o++)
				{
					//cout << o << endl;
					float d = 0;
					d = sqrt(pow((indexE[0] - index[o][0]), 2) + pow((indexE[1] - index[o][1]), 2));
					if (d < s)
					{
						s = d;
						p = o;
					}

				}
				if (s < 9)
				{
					indexConnected(indexE, index[p], outputimage7);
				}
			}
		}

		joinFilter->SetInput(i, outputimage7);

	}
	joinFilter->Update();



	//===============================洪水填充==========================================
	///*internalImageType::Pointer outputimage2 = internalImageType::New();
	//outputimage2 = outputimage1;*/
	internalImageType3D::IndexType pixelIndex;
	internalImageType3D::IndexType pixelIndex1;
	internalImageType3D::IndexType pixelIndex2;
	internalImageType3D::IndexType pixelIndex3;
	internalImageType3D::IndexType pixelIndex4;
	FloodFillNIMABI floodfill1;
	FloodFillNIMABI floodfill2;
	FloodFillNIMABI floodfill3;
	FloodFillNIMABI floodfill4;
	FloodFillNIMABI floodfill5;

	/*FloodFillNIMABI floodfill1;
	FloodFillNIMABI floodfill2;
	FloodFillNIMABI floodfill3;
	FloodFillNIMABI floodfill4;

	floodfill1.FillIterator(pixelIndex, 7, 1, joinFilter->GetOutput());

	floodfill2.FillIterator(pixelIndex1, 7, 11,  joinFilter->GetOutput());
	floodfill3.FillIterator(pixelIndex, 7, 1, joinFilter->GetOutput());
	floodfill4.FillIterator(pixelIndex1, 7, 11, joinFilter->GetOutput());*/


	//================================红色分界点归集===================================
	extractImageFilterType::Pointer extractFilter1 = extractImageFilterType::New();
	joinSeriesImageFilterType::Pointer joinFilter1 = joinSeriesImageFilterType::New();
	extractFilter1->InPlaceOn();
	extractFilter1->SetDirectionCollapseToIdentity();
	extractFilter1->SetInput(joinFilter->GetOutput());
	internalImageType3D::RegionType extractRegion1 = joinFilter->GetOutput()->GetLargestPossibleRegion();
	internalImageType3D::SizeType size1 = extractRegion1.GetSize();
	//size[2] = 1;
	internalImageType3D::IndexType index1 = extractRegion1.GetIndex();

	joinFilter1->SetOrigin(0);
	joinFilter1->SetSpacing(imageSpacing[2]);

	for (int i = 0; i < imageSize[2]; i++)
	{
		index1[2] = i;
		size1[2] = 0;
		extractRegion1.SetSize(size1);
		extractRegion1.SetIndex(index1);
		extractFilter1->SetExtractionRegion(extractRegion1);
		extractFilter1->Update();

		internalImageType::Pointer inputimage = internalImageType::New();
		inputimage = extractFilter1->GetOutput();
		internalImageType::Pointer outputimage = internalImageType::New();
		outputimage->SetRegions(extractFilter1->GetOutput()->GetRequestedRegion());
		outputimage->CopyInformation(extractFilter1->GetOutput());
		outputimage->Allocate();

		neighborhoodIteratorType::RadiusType radius;
		radius.Fill(1);

		outputimage->SetRegions(inputimage->GetRequestedRegion());
		outputimage->CopyInformation(inputimage);
		outputimage->Allocate();

		/*linearIteratorType inputit2(outputimage2, outputimage2->GetRequestedRegion());
		linearIteratorType inputit3(outputimage2, outputimage2->GetRequestedRegion());*/
		linearIteratorType inputit2(inputimage, extractFilter1->GetOutput()->GetRequestedRegion());
		linearIteratorType inputit3(inputimage, extractFilter1->GetOutput()->GetRequestedRegion());
		linearIteratorType outputit3(outputimage, outputimage->GetRequestedRegion());
		inputit3.SetDirection(0);
		outputit3.SetDirection(0);

		for (inputit3.GoToBegin(), outputit3.GoToBegin(); !inputit3.IsAtEnd(); inputit3.NextLine(), outputit3.NextLine())
		{
			inputit3.GoToBeginOfLine();
			outputit3.GoToBeginOfLine();
			while (!inputit3.IsAtEndOfLine())
			{
				float s = 0, s1 = 0, s2 = 9999, s3 = 9999;
				if (inputit3.Get() == 8)
				{
					for (inputit2.GoToBegin(); !inputit2.IsAtEnd(); inputit2.NextLine())
					{
						inputit2.GoToBeginOfLine();
						while (!inputit2.IsAtEndOfLine())
						{
							if (inputit2.Get() == 1)
							{
								s = sqrt(pow((inputit3.GetIndex()[0] - inputit2.GetIndex()[0]), 2) + pow((inputit3.GetIndex()[1] - inputit2.GetIndex()[1]), 2));
								if (s2 > s)
								{
									s2 = s;
								}
							}
							if (inputit2.Get() == 11)
							{
								s1 = sqrt(pow((inputit3.GetIndex()[0] - inputit2.GetIndex()[0]), 2) + pow((inputit3.GetIndex()[1] - inputit2.GetIndex()[1]), 2));
								if (s3 > s1)
								{
									s3 = s1;
								}
							}
							++inputit2;
						}
					}
					if (s2 != 9999 && s3 != 9999)
					{
						//if (s2 < s3)
						if (s2 < s3 || s2 == s3)
						{
							outputit3.Set(1);
						}
						//if (s2 > s3 || s2 == s3)
						if (s2 > s3)
						{
							outputit3.Set(11);
						}
					}
					else
					{
						outputit3.Set(inputit3.Get());
					}

				}
				else
					outputit3.Set(inputit3.Get());
				++inputit3;
				++outputit3;
			}
		}

		//================================黄色边界点归集===================================
		internalImageType::Pointer outputimage1 = internalImageType::New();
		outputimage1->SetRegions(inputimage->GetRequestedRegion());
		outputimage1->CopyInformation(outputimage);
		outputimage1->Allocate();

		linearIteratorType inputit5(outputimage, outputimage->GetRequestedRegion());
		linearIteratorType inputit4(outputimage, outputimage->GetRequestedRegion());
		linearIteratorType outputit4(outputimage1, outputimage1->GetRequestedRegion());
		inputit4.SetDirection(0);
		outputit4.SetDirection(0);

		for (inputit4.GoToBegin(), outputit4.GoToBegin(); !inputit4.IsAtEnd(); inputit4.NextLine(), outputit4.NextLine())
		{
			inputit4.GoToBeginOfLine();
			outputit4.GoToBeginOfLine();
			while (!inputit4.IsAtEndOfLine())
			{
				float s = 0, s1 = 0, s2 = 9999, s3 = 9999;
				if ((inputit4.Get() == 2) || (inputit4.Get() == 3) || (inputit4.Get() == 7) || (inputit4.Get() == 10))
				{
					for (inputit5.GoToBegin(); !inputit5.IsAtEnd(); inputit5.NextLine())
					{
						inputit5.GoToBeginOfLine();
						while (!inputit5.IsAtEndOfLine())
						{
							if (inputit5.Get() == 1)
							{
								s = sqrt(pow((inputit4.GetIndex()[0] - inputit5.GetIndex()[0]), 2) + pow((inputit4.GetIndex()[1] - inputit5.GetIndex()[1]), 2));
								if (s2 > s)
								{
									s2 = s;
								}
							}
							if (inputit5.Get() == 11)
							{
								s1 = sqrt(pow((inputit4.GetIndex()[0] - inputit5.GetIndex()[0]), 2) + pow((inputit4.GetIndex()[1] - inputit5.GetIndex()[1]), 2));
								if (s3 > s1)
								{
									s3 = s1;
								}
							}
							++inputit5;
						}
					}
					if (s2 != 9999 && s3 != 9999)
					{
						if (s2 < s3)
							//if (s2 < s3 || s2 == s3)
						{
							outputit4.Set(1);
						}
						if (s2 > s3 || s2 == s3)
							//if (s2 > s3)
						{
							outputit4.Set(11);
						}
					}
					else
					{
						outputit4.Set(inputit4.Get());
					}
				}
				else
					outputit4.Set(inputit4.Get());
				++inputit4;
				++outputit4;
			}
		}
		joinFilter1->SetInput(i, outputimage1);
	}
	joinFilter1->Update();

	connectedThresholdFilterType::Pointer connectedThresholdFilter1 = connectedThresholdFilterType::New();
	connectedThresholdFilter1->SetLower(1);
	connectedThresholdFilter1->SetUpper(10);
	connectedThresholdFilter1->SetInput(joinFilter1->GetOutput());
	connectedThresholdFilter1->SetReplaceValue(1);
	connectedThresholdFilter1->AddSeed(pixelIndex);
	connectedThresholdFilter1->Update();

	connectedThresholdFilterType::Pointer connectedThresholdFilter2 = connectedThresholdFilterType::New();
	connectedThresholdFilter2->SetLower(2);
	connectedThresholdFilter2->SetUpper(11);
	connectedThresholdFilter2->SetInput(joinFilter1->GetOutput());
	connectedThresholdFilter2->SetReplaceValue(11);
	connectedThresholdFilter2->AddSeed(pixelIndex1);
	connectedThresholdFilter2->Update();


	//=====================平均dice系数计算===========================================

	connectedThresholdFilterType::Pointer connectedThresholdFilter3 = connectedThresholdFilterType::New();
	connectedThresholdFilter3->SetLower(186);
	connectedThresholdFilter3->SetUpper(3071);
	connectedThresholdFilter3->SetInput(reader1->GetOutput());
	connectedThresholdFilter3->SetReplaceValue(8);
	connectedThresholdFilter3->AddSeed(pixelIndex);
	connectedThresholdFilter3->Update();

	connectedThresholdFilterType::Pointer connectedThresholdFilter4 = connectedThresholdFilterType::New();
	connectedThresholdFilter4->SetLower(186);
	connectedThresholdFilter4->SetUpper(3071);
	connectedThresholdFilter4->SetInput(reader1->GetOutput());
	connectedThresholdFilter4->SetReplaceValue(9);
	connectedThresholdFilter4->AddSeed(pixelIndex1);
	connectedThresholdFilter4->Update();

	internalImageType3D::Pointer outputimage3 = internalImageType3D::New();
	outputimage3 = connectedThresholdFilter1->GetOutput();
	outputimage3->SetRegions(connectedThresholdFilter1->GetOutput()->GetRequestedRegion());
	outputimage3->CopyInformation(connectedThresholdFilter1->GetOutput());
	outputimage3->Allocate();
	internalImageType3D::Pointer outputimage4 = internalImageType3D::New();
	outputimage4 = connectedThresholdFilter2->GetOutput();
	outputimage4->SetRegions(connectedThresholdFilter2->GetOutput()->GetRequestedRegion());
	outputimage4->CopyInformation(connectedThresholdFilter2->GetOutput());
	outputimage4->Allocate();
	internalImageType3D::Pointer outputimage5 = internalImageType3D::New();
	outputimage5 = connectedThresholdFilter3->GetOutput();
	outputimage5->SetRegions(connectedThresholdFilter3->GetOutput()->GetRequestedRegion());
	outputimage5->CopyInformation(connectedThresholdFilter3->GetOutput());
	outputimage5->Allocate();
	internalImageType3D::Pointer outputimage6 = internalImageType3D::New();
	outputimage6 = connectedThresholdFilter4->GetOutput();
	outputimage6->SetRegions(connectedThresholdFilter4->GetOutput()->GetRequestedRegion());
	outputimage6->CopyInformation(connectedThresholdFilter4->GetOutput());
	outputimage6->Allocate();

	auto projectionDirection = static_cast<unsigned char>(2);
	unsigned int i, j;
	unsigned int direction[2];
	for (i = 0, j = 0; i < 3; ++i)
	{
		if (i != projectionDirection)
		{
			direction[j] = i;
			j++;
		}
	}

	slicerIteratorType inputit9(outputimage3, outputimage3->GetRequestedRegion());
	inputit9.SetFirstDirection(direction[1]);
	inputit9.SetSecondDirection(direction[0]);
	slicerIteratorType inputit10(outputimage4, outputimage4->GetRequestedRegion());
	inputit10.SetFirstDirection(direction[1]);
	inputit10.SetSecondDirection(direction[0]);
	slicerIteratorType inputit11(outputimage5, outputimage5->GetRequestedRegion());
	inputit11.SetFirstDirection(direction[1]);
	inputit11.SetSecondDirection(direction[0]);
	slicerIteratorType inputit12(outputimage6, outputimage6->GetRequestedRegion());
	inputit12.SetFirstDirection(direction[1]);
	inputit12.SetSecondDirection(direction[0]);


	//float pic = 0;
	//float jiaobingbihe = 0;
	//inputit11.GoToBegin();
	//inputit9.GoToBegin();
	//while (!inputit11.IsAtEnd())
	//{
	//	float tongji = 0, shoudong8 = 0, zidong8 = 0;
	//	float jiaobingbi = 0;
	//	while (!inputit11.IsAtEndOfSlice())
	//	{
	//		while (!inputit11.IsAtEndOfLine())
	//		{
	//			//if ((inputit9.Get() == 8)|| (inputit9.Get() == 7)|| (inputit9.Get() == 6)|| (inputit9.Get() == 5)||(inputit9.Get() == 4) || (inputit9.Get() == 3) || (inputit9.Get() == 2) || (inputit9.Get() == 1) || (inputit9.Get() == 9) || (inputit9.Get() == 10) || (inputit9.Get() == 11))
	//			if (inputit11.Get() == 8 && inputit9.Get() == 1)
	//			{
	//				tongji++;
	//				//cout << "inputit9:" << inputit9.Get() << endl;
	//				/*if (inputit9.Get() == 1)
	//					jiao++;*/
	//			}
	//			if (inputit9.Get() == 1)
	//			{
	//				zidong8++;
	//			}
	//			if (inputit11.Get() == 8)
	//			{
	//				shoudong8++;
	//			}
	//			++inputit11;
	//			++inputit9;
	//		}
	//		inputit11.NextLine();
	//		inputit9.NextLine();
	//	}
	//	
	//	inputit11.NextSlice();
	//	inputit9.NextSlice();
	//	if (zidong8 != 0)
	//	{
	//		jiaobingbi = tongji / (zidong8 + shoudong8 - tongji);
	//		pic++;
	//		jiaobingbihe = jiaobingbihe + jiaobingbi;
	//		cout << "done" << endl;
	//	}

	//	cout << setiosflags(ios::fixed) << setprecision(4);
	//	cout << "tongji:" << tongji << endl;
	//	cout << "zidong8:" << zidong8 << endl;
	//	cout << "shoudong8:" << shoudong8 << endl;
	//	cout << "jiaobingbi:" << jiaobingbi << endl;
	//	cout << "  " << endl;
	//}
	//float pingjun = 0;
	//pingjun = jiaobingbihe / pic;
	//
	//cout << "pic:" << pic<< endl;
	//cout << "pingjun:" << pingjun << endl;

	float diceave8 = 0.0, diceave9 = 0.0;
	float dicehe8 = 0.0, dicehe9 = 0.0;
	float pic1 = 0, pic2 = 0;

	float senave8 = 0.0, senave9 = 0.0;
	float senhe8 = 0.0, senhe9 = 0.0;

	float specave8 = 0.0, specave9 = 0.0;
	float speche8 = 0.0, speche9 = 0.0;

	float preave8 = 0.0, preave9 = 0.0;
	float prehe8 = 0.0, prehe9 = 0.0;

	float F1ave8 = 0.0, F1ave9 = 0.0;
	float F1he8 = 0.0, F1he9 = 0.0;

	float accave8 = 0.0, accave9 = 0.0;
	float acche8 = 0.0, acche9 = 0.0;

	float HDave8 = 0.0, HDave9 = 0.0;
	float HDhe8 = 0.0, HDhe9 = 0.0;

	//float recave8 = 0.0, recave9 = 0.0;
	//float reche8 = 0.0, reche9 = 0.0;

	inputit9.GoToBegin();
	inputit10.GoToBegin();
	inputit11.GoToBegin();
	inputit12.GoToBegin();

	internalImageType::RegionType readerRegion = reader2d->GetOutput()->GetLargestPossibleRegion();

	FILE * fp;
	errno_t err;
	err = fopen_s(&fp, "file.txt", "w+");
	float pic = 0.0;

	FILE * fp1;
	errno_t err1;
	err1 = fopen_s(&fp1, "fileP.txt", "w+");

	FILE * fp2;
	errno_t err2;
	err2 = fopen_s(&fp2, "fileD.txt", "w+");

	while (!inputit9.IsAtEnd())
	{
		float dicejiao8 = 0, dicejiao9 = 0, dice18 = 0, dice28 = 0, dice19 = 0, dice29 = 0;
		float dice9 = 0.0, dice8 = 0.0;
		float sen9 = 0.0, sen8 = 0.0;
		float spec9 = 0.0, spec8 = 0.0;
		float pre9 = 0.0, pre8 = 0.0;
		//float rec9 = 0.0, rec8 = 0.0;
		float F19 = 0.0, F18 = 0.0;
		float acc9 = 0.0, acc8 = 0.0;
		float HD9 = 0.0, HD8 = 0.0;

		float pixelnumber = 0.0;
		float TN8 = 0.0, TN9 = 0.0;

		float Hausdoffdis = 0.0;

		hausdorffDistanceFilterType::Pointer hausdorffFilter = hausdorffDistanceFilterType::New();
		hausdorffDistanceFilterType::Pointer hausdorffFilter1 = hausdorffDistanceFilterType::New();
		internalImageType::Pointer hausImage1 = internalImageType::New();
		hausImage1->SetRegions(readerRegion);
		hausImage1->CopyInformation(hausImage1);
		hausImage1->Allocate();
		internalImageType::Pointer hausImage2 = internalImageType::New();
		hausImage2->SetRegions(readerRegion);
		hausImage2->CopyInformation(hausImage1);
		hausImage2->Allocate();
		internalImageType::Pointer hausImage3 = internalImageType::New();
		hausImage3->SetRegions(readerRegion);
		hausImage3->CopyInformation(hausImage1);
		hausImage3->Allocate();
		internalImageType::Pointer hausImage4 = internalImageType::New();
		hausImage4->SetRegions(readerRegion);
		hausImage4->CopyInformation(hausImage1);
		hausImage4->Allocate();
		linearIteratorType hausinputit1(hausImage1, hausImage1->GetRequestedRegion());
		linearIteratorType hausinputit2(hausImage2, hausImage2->GetRequestedRegion());
		linearIteratorType hausinputit3(hausImage3, hausImage3->GetRequestedRegion());
		linearIteratorType hausinputit4(hausImage4, hausImage4->GetRequestedRegion());

		hausinputit1.GoToBegin();
		hausinputit2.GoToBegin();
		hausinputit3.GoToBegin();
		hausinputit4.GoToBegin();

		while (!inputit9.IsAtEndOfSlice())
		{
			while (!inputit9.IsAtEndOfLine())
			{
				if (inputit9.Get() == 1 && inputit11.Get() == 8)
				{
					dicejiao8++;
				}
				if (inputit10.Get() == 11 && inputit12.Get() == 9)
				{
					dicejiao9++;
				}
				if (inputit9.Get() == 1)
				{
					dice18++;
					hausinputit1.Set(1);
				}
				if (inputit11.Get() == 8)
				{
					dice28++;
					hausinputit2.Set(1);
				}
				if (inputit10.Get() == 11)
				{
					dice19++;
					hausinputit3.Set(1);
				}
				if (inputit12.Get() == 9)
				{
					dice29++;
					hausinputit4.Set(1);
				}

				pixelnumber++;

				++inputit9;
				++inputit10;
				++inputit11;
				++inputit12;
				++hausinputit1;
				++hausinputit2;
				++hausinputit3;
				++hausinputit4;
			}
			inputit9.NextLine();
			inputit10.NextLine();
			inputit11.NextLine();
			inputit12.NextLine();
			hausinputit1.NextLine();
			hausinputit2.NextLine();
			hausinputit3.NextLine();
			hausinputit4.NextLine();
		}
		inputit9.NextSlice();
		inputit10.NextSlice();
		inputit11.NextSlice();
		inputit12.NextSlice();

		pixelnumber = pixelnumber * 0.002;

		TN8 = pixelnumber - dice18 - dice28 + dicejiao8;
		TN9 = pixelnumber - dice19 - dice29 + dicejiao9;

		if (dice18 != 0)
		{
			dice8 = (dicejiao8) / (dice18 + dice28 - dicejiao8);
			sen8 = (dicejiao8) / (dice28);
			spec8 = (TN8) / (pixelnumber - dice28);
			pre8 = (dicejiao8) / dice18;
			//rec8= (dicejiao8) / dice28;
			acc8 = (dicejiao8 + TN8) / (pixelnumber);
			F18 = (2 * acc8*sen8) / (acc8 + sen8);
			pic1++;
		}

		if (dice19 != 0)
		{
			dice9 = (dicejiao9) / (dice19 + dice29 - dicejiao9);
			sen9 = (dicejiao9) / (dice29);
			spec9 = (TN9) / (pixelnumber - dice29);
			pre9 = (dicejiao9) / dice19;
			//rec9 = (dicejiao9) / dice29;
			acc9 = (dicejiao9 + TN9) / (pixelnumber);
			F19 = (2 * acc9*sen9) / (acc9 + sen9);
			pic2++;
		}


		if (dicejiao8 != 0)
		{
			hausdorffFilter->SetInput1(hausImage1);
			hausdorffFilter->SetInput2(hausImage2);
			hausdorffFilter->Update();
		}

		if (dicejiao9 != 0)
		{
			hausdorffFilter1->SetInput1(hausImage3);
			hausdorffFilter1->SetInput2(hausImage4);
			hausdorffFilter1->Update();
		}

		HD8 = hausdorffFilter->GetHausdorffDistance();
		HD9 = hausdorffFilter1->GetHausdorffDistance();

		if (HD8 > HD9)
		{
			Hausdoffdis = HD8;
		}
		else
		{
			Hausdoffdis = HD9;
		}

		pic++;

		cout << setiosflags(ios::fixed) << setprecision(4);
		cout << "第" << pic1 << "切片近端交并比：" << dice8 << endl;
		cout << "第" << pic2 << "切片远端交并比：" << dice9 << endl;
		cout << "第" << pic1 << "切片近端灵敏度：" << sen8 << endl;
		cout << "第" << pic2 << "切片远端灵敏度：" << sen9 << endl;
		cout << "第" << pic1 << "切片近端特异率：" << spec8 << endl;
		cout << "第" << pic2 << "切片远端特异率：" << spec9 << endl;
		cout << "第" << pic1 << "切片近端精准度：" << pre8 << endl;
		cout << "第" << pic2 << "切片远端精准度：" << pre9 << endl;
		//cout << "第" << pic1 << "切片近端召回率：" << rec8 << endl;
		//cout << "第" << pic2 << "切片远端召回率：" << rec9 << endl;
		cout << "第" << pic1 << "切片近端准确率：" << acc8 << endl;
		cout << "第" << pic2 << "切片远端准确率：" << acc9 << endl;
		cout << "第" << pic1 << "切片近端F1系数：" << F18 << endl;
		cout << "第" << pic2 << "切片远端F1系数：" << F19 << endl;
		cout << pixelnumber << endl;
		cout << dice28 << endl;
		cout << dice29 << endl;
		cout << " " << endl;

		dicehe8 = dicehe8 + dice8;
		dicehe9 = dicehe9 + dice9;
		senhe8 = senhe8 + sen8;
		senhe9 = senhe9 + sen9;
		speche8 = speche8 + spec8;
		speche9 = speche9 + spec9;
		prehe8 = prehe8 + pre8;
		prehe9 = prehe9 + pre9;
		//reche8 = reche8 + rec8;
		//reche9 = reche9 + rec9;
		acche8 = acche8 + acc8;
		acche9 = acche9 + acc9;
		F1he8 = F1he8 + F18;
		F1he9 = F1he9 + F19;
		HDhe8 = HDhe8 + HD8;
		HDhe9 = HDhe9 + HD9;

		fprintf(fp, "%f\t%f\n", Hausdoffdis, pic);
		fprintf(fp1, "%f\t%f\n", HD8, pic1);
		fprintf(fp2, "%f\t%f\n", HD9, pic2);
	}

	diceave8 = dicehe8 / pic1;
	diceave9 = dicehe9 / pic2;
	senave8 = senhe8 / pic1;
	senave9 = senhe9 / pic2;
	specave8 = speche8 / pic1;
	specave9 = speche9 / pic2;
	preave8 = prehe8 / pic1;
	preave9 = prehe9 / pic2;
	//recave8 = reche8 / pic1;
	//recave9 = reche9 / pic2;
	accave8 = acche8 / pic1;
	accave9 = acche9 / pic2;
	F1ave8 = F1he8 / pic1;
	F1ave9 = F1he9 / pic2;
	HDave8 = HDhe8 / pic1;
	HDave9 = HDhe9 / pic2;

	cout << "近端平均交并比：" << diceave8 << endl;
	cout << "远端平均交并比：" << diceave9 << endl;
	cout << "近端平均灵敏度：" << senave8 << endl;
	cout << "远端平均灵敏度：" << senave9 << endl;
	cout << "近端平均特异率：" << specave8 << endl;
	cout << "远端平均特异率：" << specave9 << endl;
	cout << "近端平均精准度：" << preave8 << endl;
	cout << "远端平均精准度：" << preave9 << endl;
	//cout << "近端平均召回率：" << recave8 << endl;
	//cout << "远端平均召回率：" << recave9 << endl;
	cout << "近端平均准确率：" << accave8 << endl;
	cout << "远端平均准确率：" << accave9 << endl;
	cout << "近端平均F1系数：" << F1ave8 << endl;
	cout << "远端平均F1系数：" << F1ave9 << endl;
	cout << "近端平均HD系数：" << HDave8 << endl;
	cout << "远端平均HD系数：" << HDave9 << endl;

	cout << " " << endl;

	vtkSmartPointer<vtkLookupTable> colorTable = vtkSmartPointer<vtkLookupTable>::New();
	colorTable->SetNumberOfColors(12);
	colorTable->SetTableRange(0, 11);
	colorTable->SetTableValue(0, 0, 0, 0, 0);
	colorTable->SetTableValue(1, 0.5, 0.5, 1, 1);
	colorTable->SetTableValue(2, 1, 1, 0, 1);
	colorTable->SetTableValue(3, 0, 1, 0, 1);
	colorTable->SetTableValue(4, 0, 1, 1, 1);
	colorTable->SetTableValue(5, 0, 0, 1, 1);
	colorTable->SetTableValue(6, 1, 0, 1, 1);
	colorTable->SetTableValue(7, 1, 0.5, 0.5, 1);//粉色
	colorTable->SetTableValue(8, 1, 0, 0, 1);
	colorTable->SetTableValue(9, 1, 0.2, 0.5, 1);
	colorTable->SetTableValue(10, 1, 1, 1, 1);
	colorTable->SetTableValue(11, 0.5, 1, 0.5, 1);
	colorTable->Build();

	itkToVTKimageFilterType::Pointer itkToVtkFilter = itkToVTKimageFilterType::New();
	itkToVtkFilter->SetInput(joinFilter1->GetOutput());
	itkToVtkFilter->Update();

	itkToVTKimageFilterType::Pointer itkToVtkFilter1 = itkToVTKimageFilterType::New();
	itkToVtkFilter1->SetInput(connectedThresholdFilter1->GetOutput());
	itkToVtkFilter1->Update();

	itkToVTKimageFilterType::Pointer itkToVtkFilter2 = itkToVTKimageFilterType::New();
	itkToVtkFilter2->SetInput(connectedThresholdFilter2->GetOutput());
	itkToVtkFilter2->Update();

	itkToVTKimageFilterType::Pointer itkToVtkFilter3 = itkToVTKimageFilterType::New();
	itkToVtkFilter3->SetInput(connectedThresholdFilter3->GetOutput());
	itkToVtkFilter3->Update();

	itkToVTKimageFilterType::Pointer itkToVtkFilter4 = itkToVTKimageFilterType::New();
	itkToVtkFilter4->SetInput(joinFilter->GetOutput());
	itkToVtkFilter4->Update();

	//=============================分割导出=========================
	writerType3D::Pointer writer1 = writerType3D::New();
	/*NameGenerator->SetOutputDirectory("E:\\SE2");
	writer1->SetMetaDataDictionaryArray(reader->GetMetaDataDictionaryArray());
	writer1->SetImageIO(DicomIO);
	writer1->SetFileNames(NameGenerator->GetOutputFileNames());
	writer1->SetInput(connectedThresholdFilter1->GetOutput());
	std::cout << "erro" << std::endl;
	writer1->Update();
	std::cout << "erro1" << std::endl;*/

	NamesGeneratorType1::Pointer nameGenerator1 = NamesGeneratorType1::New();
	std::string format = "E:\\SE2\\";//文件前缀
	format += "%03d.";//文件序列规则
	format += "dcm"; // 文件格式
	nameGenerator1->SetSeriesFormat(format.c_str());

	nameGenerator1->SetStartIndex(index[2]);
	nameGenerator1->SetEndIndex(index[2]+size[2]/2-60);
	nameGenerator1->SetIncrementIndex(1);
	writer1->SetImageIO(DicomIO);
	writer1->SetFileNames(nameGenerator1->GetFileNames());
	writer1->SetInput(connectedThresholdFilter1->GetOutput());
	writer1->Update();
	//
	/*writerType3D::Pointer writer2 = writerType3D::New();
	NameGenerator->SetOutputDirectory("F:\\SE3");
	writer2->SetMetaDataDictionaryArray(reader->GetMetaDataDictionaryArray());
	writer2->SetImageIO(DicomIO);
	writer2->SetFileNames(NameGenerator->GetOutputFileNames());
	writer2->SetInput(connectedThresholdFilter2->GetOutput());
	writer2->Update();*/


	vtkSmartPointer<vtkImageViewer2> viewer3D = vtkSmartPointer<vtkImageViewer2>::New();
	viewer3D->SetInputData(itkToVtkFilter->GetOutput());
	viewer3D->SetSliceOrientationToXY();
	viewer3D->GetImageActor()->GetProperty()->UseLookupTableScalarRangeOn();
	viewer3D->GetImageActor()->SetInterpolate(false);
	viewer3D->GetImageActor()->GetProperty()->SetLookupTable(colorTable);
	viewer3D->GetImageActor()->GetProperty()->SetDiffuse(0);
	viewer3D->GetImageActor()->SetPickable(false);

	vtkSmartPointer<vtkRenderWindowInteractor> rwi = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkInteractorStyleImage> style = vtkSmartPointer<vtkInteractorStyleImage>::New();
	rwi->SetInteractorStyle(style);
	viewer3D->SetupInteractor(rwi);

	vtkSmartPointer<vtkSliderRepresentation2D> sliderRep = vtkSmartPointer<vtkSliderRepresentation2D>::New();
	sliderRep->SetMinimumValue(viewer3D->GetSliceMin());
	sliderRep->SetMaximumValue(viewer3D->GetSliceMax());
	sliderRep->SetValue(5.0);
	sliderRep->GetSliderProperty()->SetColor(1, 0, 0);
	sliderRep->GetTitleProperty()->SetColor(1, 0, 0);
	sliderRep->GetLabelProperty()->SetColor(1, 0, 0);
	sliderRep->GetSelectedProperty()->SetColor(0, 1, 0);
	sliderRep->GetTubeProperty()->SetColor(1, 1, 0);
	sliderRep->GetCapProperty()->SetColor(1, 1, 0);
	sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToDisplay();
	sliderRep->GetPoint1Coordinate()->SetValue(40, 40);
	sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToDisplay();
	sliderRep->GetPoint2Coordinate()->SetValue(1000, 40);

	vtkSmartPointer<vtkSliderWidget> sliderWidget = vtkSmartPointer<vtkSliderWidget>::New();
	sliderWidget->SetInteractor(rwi);
	sliderWidget->SetRepresentation(sliderRep);
	sliderWidget->SetAnimationModeToAnimate();
	sliderWidget->EnabledOn();

	vtkSmartPointer<vtkSliderCallback> callback = vtkSmartPointer<vtkSliderCallback>::New();
	callback->Viewer = viewer3D;
	sliderWidget->AddObserver(vtkCommand::InteractionEvent, callback);
	rwi->Start();

	//=====================================骨折区域重建========================================
	//======================远端==============================
	//vtkSmartPointer<vtkGPUVolumeRayCastMapper> LodMapper1 = vtkSmartPointer<vtkGPUVolumeRayCastMapper>::New();
	//LodMapper1->SetInputData(itkToVtkFilter1->GetOutput());
	//LodMapper1->SetAutoAdjustSampleDistances(1);

	//vtkSmartPointer<vtkGPUVolumeRayCastMapper> Mapper1 = vtkSmartPointer<vtkGPUVolumeRayCastMapper>::New();
	//Mapper1->SetInputData(itkToVtkFilter1->GetOutput());
	//Mapper1->SetAutoAdjustSampleDistances(0);
	//Mapper1->SetSampleDistance(4 * Mapper1->GetSampleDistance());
	//Mapper1->SetImageSampleDistance(4 * Mapper1->GetImageSampleDistance());

	//vtkSmartPointer<vtkPiecewiseFunction> GradientOpacity1 = vtkSmartPointer<vtkPiecewiseFunction>::New();
	//GradientOpacity1->AddPoint(0, 0.0);
	//GradientOpacity1->AddPoint(1, 1);
	////GradientOpacity1->AddPoint(5, 0.5);
	////GradientOpacity1->AddPoint(8, 0.4);
	//GradientOpacity1->SetClamping(true);

	//vtkSmartPointer<vtkPiecewiseFunction> CompositeOpacity1 = vtkSmartPointer<vtkPiecewiseFunction>::New();
	//CompositeOpacity1->AddPoint(0, 0.0);
	//CompositeOpacity1->AddPoint(1, 1);
	////CompositeOpacity1->AddPoint(5, 0.4);
	////CompositeOpacity1->AddPoint(8, 0.6);
	//CompositeOpacity1->SetClamping(true);

	//vtkSmartPointer<vtkColorTransferFunction> ColorTransfer1 = vtkSmartPointer<vtkColorTransferFunction>::New();
	//ColorTransfer1->AddRGBPoint(0, 0, 0, 0);
	//ColorTransfer1->AddRGBPoint(3, 1.00, 0.5, 0.2);
	///*ColorTransfer1->AddRGBPoint(9, 0, 0, 0);
	//ColorTransfer1->AddRGBPoint(10, 0.8, 0.8, 0.8);*/

	//vtkSmartPointer<vtkVolumeProperty> Property1 = vtkSmartPointer<vtkVolumeProperty>::New();
	//Property1->SetInterpolationTypeToLinear();
	//Property1->ShadeOn();
	//Property1->SetAmbient(0.3);
	//Property1->SetSpecular(0.7);
	//Property1->SetDiffuse(0.4);

	//Property1->SetGradientOpacity(GradientOpacity1);
	//Property1->SetScalarOpacity(CompositeOpacity1);
	//Property1->SetColor(ColorTransfer1);

	//vtkSmartPointer<vtkLODProp3D> Prop1 = vtkSmartPointer<vtkLODProp3D>::New();
	//Prop1->AddLOD(LodMapper1, Property1, 0);
	//Prop1->AddLOD(Mapper1, Property1, 0);
	////==================================近端====================================
	//vtkSmartPointer<vtkGPUVolumeRayCastMapper> LodMapper2 = vtkSmartPointer<vtkGPUVolumeRayCastMapper>::New();
	//LodMapper2->SetInputData(itkToVtkFilter2->GetOutput());
	//LodMapper2->SetAutoAdjustSampleDistances(1);

	//vtkSmartPointer<vtkGPUVolumeRayCastMapper> Mapper2 = vtkSmartPointer<vtkGPUVolumeRayCastMapper>::New();
	//Mapper2->SetInputData(itkToVtkFilter2->GetOutput());
	//Mapper2->SetAutoAdjustSampleDistances(0);
	//Mapper2->SetSampleDistance(4 * Mapper2->GetSampleDistance());
	//Mapper2->SetImageSampleDistance(4 * Mapper2->GetImageSampleDistance());

	//vtkSmartPointer<vtkPiecewiseFunction> GradientOpacity2 = vtkSmartPointer<vtkPiecewiseFunction>::New();
	//GradientOpacity2->AddPoint(0, 0.0);
	////GradientOpacity2->AddPoint(1, 0.8);
	////GradientOpacity2->AddPoint(5, 0.5);
	//GradientOpacity2->AddPoint(11, 1);
	//GradientOpacity2->SetClamping(true);

	//vtkSmartPointer<vtkPiecewiseFunction> CompositeOpacity2 = vtkSmartPointer<vtkPiecewiseFunction>::New();
	//CompositeOpacity2->AddPoint(0, 0.0);
	///*CompositeOpacity2->AddPoint(1, 0.8);
	//CompositeOpacity2->AddPoint(5, 0.4);*/
	//CompositeOpacity2->AddPoint(11, 1);
	//CompositeOpacity2->SetClamping(true);

	//vtkSmartPointer<vtkColorTransferFunction> ColorTransfer2 = vtkSmartPointer<vtkColorTransferFunction>::New();
	//ColorTransfer2->AddRGBPoint(0, 0, 0, 0);
	//ColorTransfer2->AddRGBPoint(13, 0.8, 0.8, 0.8);
	////ColorTransfer2->AddRGBPoint(9, 0, 0, 0);
	////ColorTransfer2->AddRGBPoint(12, 0.8, 0.8, 0.8);

	//vtkSmartPointer<vtkVolumeProperty> Property2 = vtkSmartPointer<vtkVolumeProperty>::New();
	//Property2->SetInterpolationTypeToLinear();
	//Property2->ShadeOn();
	//Property2->SetAmbient(0.3);
	//Property2->SetSpecular(0.7);
	//Property2->SetDiffuse(0.4);

	//Property2->SetGradientOpacity(GradientOpacity2);
	//Property2->SetScalarOpacity(CompositeOpacity2);
	//Property2->SetColor(ColorTransfer2);

	//vtkSmartPointer<vtkLODProp3D> Prop2 = vtkSmartPointer<vtkLODProp3D>::New();
	//Prop2->AddLOD(LodMapper2, Property2, 0);
	//Prop2->AddLOD(Mapper2, Property2, 0);




	////==================================渲染=====================================
	//vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
	//Renderer->AddVolume(Prop1);
	//Renderer->AddVolume(Prop2);
	//Renderer->SetBackground(0.2, 0.6, 0.5);
	//Renderer->SetViewport(0, 0, 0.5, 1);
	//
	//vtkSmartPointer<vtkRenderWindow>  renWindow = vtkRenderWindow::New();
	//renWindow->SetSize(1000, 1000);
	//renWindow->AddRenderer(Renderer);
	//	
	//vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkRenderWindowInteractor::New();
	//iren->SetRenderWindow(renWindow);
	//vtkSmartPointer<vtkInteractorStyleTrackballCamera> Style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	//iren->SetInteractorStyle(Style);
	//
	//Renderer->ResetCamera();
	//renWindow->Render();
	//iren->Initialize();
	//iren->Start();


	return 0;
}

