// components/main_ui.js
'use client';
import { useState } from 'react';
import { Button } from "@/components/ui/button";
import Image from "next/image";

export default function MainUI() {
  const [selectedFile, setSelectedFile] = useState(null);

  const handleFileChange = (event) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file);
    }
  };  

  const handleUpload = () => {
    // You can perform upload logic here, e.g., send file to a server
    if (selectedFile) {
      console.log("Uploading file:", selectedFile);
      // Example: Upload file using fetch or other API
    } else {
      console.log("No file selected");
    }
  };

  return (
    <div className="flex flex-col items-center justify-center h-screen">
      <div className="bg-white dark:bg-gray-900 rounded-lg shadow-lg p-8 w-full max-w-sm">
        <div className="flex flex-col items-center space-y-6">
          <div className="flex items-center space-x-2">
            <BotIcon className="h-6 w-6 text-gray-500 dark:text-gray-400" />
            <p className="text-gray-900 dark:text-gray-200 font-medium">SLAM ROS2 Bot Talker</p>
          </div>
          <div className="relative">
            {selectedFile ? (
              <img src={URL.createObjectURL(selectedFile)} width={300} height={300} alt="Uploaded" className="rounded-lg shadow-lg" />
            ) : (
              <Image src="/kitchen.jpg" width={300} height={300} alt="Placeholder" className="rounded-lg shadow-lg" />
            )}
          </div>
          <div className="flex items-center space-x-2">
            <input type="file" accept="image/*" onChange={handleFileChange} className="hidden" id="upload-input" />
            <label htmlFor="upload-input" className="cursor-pointer">
              <Button variant="outline" className="bg-[#5856d6] text-white hover:bg-[#ff2d55] transition-colors bg-gradient-to-br from-[#5856d6] to-[#ff2d55] dark:bg-gradient-to-br dark:from-[#ff2d55] dark:to-[#5856d6]">
                <UploadIcon  onClick={handleUpload} className="mr-2 h-4 w-4 " />
                Upload Image
              </Button>
            </label>
          </div>
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

export function BotIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M12 8V4H8" />
      <rect width="16" height="12" x="4" y="8" rx="2" />
      <path d="M2 14h2" />
      <path d="M20 14h2" />
      <path d="M15 13v2" />
      <path d="M9 13v2" />
    </svg>
  )
}

export function SpeakerIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect width="16" height="20" x="4" y="2" rx="2" />
      <path d="M12 6h.01" />
      <circle cx="12" cy="14" r="4" />
      <path d="M12 14h.01" />
    </svg>
  )
}

export function UploadIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
      <polyline points="17 8 12 3 7 8" />
      <line x1="12" x2="12" y1="3" y2="15" />
    </svg>
  )
}
