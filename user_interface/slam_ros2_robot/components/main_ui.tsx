"use client";
import { useState, useRef } from "react";
import { Button } from "@/components/ui/button";
import Image from "next/image";
import { BotIcon, SpeakerIcon, UploadIcon, RecordIcon } from "./icons";
import { CameraIcon } from "lucide-react";

export default function MainUI() {
  const [selectedFile, setSelectedFile] = useState(null);
  const [isRecording, setIsRecording] = useState(false);
  const [isLiveFeed, setIsLiveFeed] = useState(false);
  const videoRef = useRef<HTMLVideoElement>(null);

  const handleFileChange = (event: any) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file);
    }
  };

  const handleUpload = () => {
    if (selectedFile) {
      console.log("Uploading file:", selectedFile);
    } else {
      console.log("No file selected");
    }
  };

  const toggleRecording = () => {
    setIsRecording((prev) => !prev);
  };

  const toggleLiveFeed = async () => {
    if (isLiveFeed) {
      // Stop live feed
      setIsLiveFeed(false);
      if (videoRef.current) {
        const stream = (videoRef.current as HTMLVideoElement)?.srcObject as MediaStream;
        if (stream) {
          const tracks = stream.getTracks();
          tracks.forEach((track) => track.stop());
        }
        (videoRef.current as HTMLVideoElement).srcObject = null;
      }
    } else {
      // Start live feed
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: true });
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          videoRef.current.play();
        }
        setIsLiveFeed(true);
      } catch (error) {
        console.error("Error accessing webcam:", error);
      }
    }
  };

  return (
    <div className="flex flex-col items-center justify-center h-screen">
      <div className="bg-white dark:bg-gray-900 rounded-lg shadow-lg p-8 w-full max-w-sm">
        <div className="flex flex-col items-center space-y-6">
          <div className="flex items-center space-x-2">
            <BotIcon className="h-6 w-6 text-gray-500 dark:text-gray-400" />
            <p className="text-gray-900 dark:text-gray-200 font-medium">
              SLAM ROS2 Bot Talker
            </p>
          </div>
          <div className="relative">
            {selectedFile ? (
              <img
                src={URL.createObjectURL(selectedFile)}
                width={300}
                height={300}
                alt="Uploaded"
                className="rounded-lg shadow-lg"
              />
            ) : (
              <Image
                src="/kitchen.jpg"
                width={300}
                height={300}
                alt="Placeholder"
                className="rounded-lg shadow-lg"
              />
            )}
          </div>
          <div className="flex items-center space-x-2">
            <input
              type="file"
              accept="image/*"
              onChange={handleFileChange}
              className="hidden"
              id="upload-input"
            />
            <label htmlFor="upload-input" className="cursor-pointer">
              <div className="flex items-center space-x-2 mb-5">
                <Button
                  variant="outline"
                  className="bg-[#5856d6] text-white hover:bg-[#ff2d55] transition-colors bg-gradient-to-br from-[#5856d6] to-[#ff2d55] dark:bg-gradient-to-br dark:from-[#ff2d55] dark:to-[#5856d6] mr-5"
                >
                  <UploadIcon onClick={handleUpload} className="mr-2 h-4 w-4" />
                  Upload Image
                </Button>
                <Button
                  variant="outline"
                  onClick={toggleLiveFeed}
                  className="bg-[#5856d6] text-white hover:bg-[#ff2d55] transition-colors bg-gradient-to-br from-[#5856d6] to-[#ff2d55] dark:bg-gradient-to-br dark:from-[#ff2d55] dark:to-[#5856d6]"
                >
                  <CameraIcon className="mr-2 h-4 w-4" />
                  {isLiveFeed ? "Stop Live Feed" : "Live Feed"}
                </Button>
              </div>
              <div className="flex items-center justify-center space-x-2">
                <Button
                  variant="outline"
                  onClick={toggleRecording}
                  className={`transition-colors ${
                    isRecording
                      ? "bg-green-500 text-white hover:bg-green-600"
                      : "bg-[#db3030] text-white hover:bg-[#ec0d0d]"
                  }`}
                >
                  <RecordIcon className="mr-2 h-4 w-4" />
                  {isRecording ? "Recording..." : "Record"}
                </Button>
              </div>
            </label>
          </div>
          {isLiveFeed && (
            <div className="w-full">
              <video
                ref={videoRef}
                className="rounded-lg shadow-lg w-full"
                autoPlay
                playsInline
              />
            </div>
          )}
          <div className="flex items-center space-x-4">
            <SpeakerIcon className="h-8 w-8 text-gray-500 dark:text-gray-400" />
            <p className="text-gray-900 dark:text-gray-200 font-medium">
              Talk to your robot, Tell what you need.
            </p>
          </div>
          <p className="text-gray-900 dark:text-gray-500 font-small text-center">
            We have used ROS2, Mistral 7B LLM, BERT, Whisper STT and many more.
          </p>
        </div>
      </div>
    </div>
  );
}