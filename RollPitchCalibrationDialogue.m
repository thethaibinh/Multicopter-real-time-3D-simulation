function varargout = RollPitchCalibrationDialogue(varargin)
% ROLLPITCHCALIBRATIONDIALOGUE MATLAB code for RollPitchCalibrationDialogue.fig
%      ROLLPITCHCALIBRATIONDIALOGUE, by itself, creates a new ROLLPITCHCALIBRATIONDIALOGUE or raises the existing
%      singleton*.
%
%      H = ROLLPITCHCALIBRATIONDIALOGUE returns the handle to a new ROLLPITCHCALIBRATIONDIALOGUE or the handle to
%      the existing singleton*.
%
%      ROLLPITCHCALIBRATIONDIALOGUE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROLLPITCHCALIBRATIONDIALOGUE.M with the given input arguments.
%
%      ROLLPITCHCALIBRATIONDIALOGUE('Property','Value',...) creates a new ROLLPITCHCALIBRATIONDIALOGUE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RollPitchCalibrationDialogue_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RollPitchCalibrationDialogue_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RollPitchCalibrationDialogue

% Last Modified by GUIDE v2.5 17-Aug-2015 15:35:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @RollPitchCalibrationDialogue_OpeningFcn, ...
    'gui_OutputFcn',  @RollPitchCalibrationDialogue_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before RollPitchCalibrationDialogue is made visible.
function RollPitchCalibrationDialogue_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RollPitchCalibrationDialogue wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = RollPitchCalibrationDialogue_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in calibOK.
function calibOK_Callback(hObject, eventdata, handles)
global ChannelOffset rc_in
ChannelOffset(1) = rc_in(1);
ChannelOffset(2) = rc_in(2);
ChannelOffset(3) = rc_in(3);
ChannelOffset(4) = rc_in(4);
set_param('multi_model/Signal Receiver/ch1_offset', 'Value', sprintf('%d', ChannelOffset(1)));
set_param('multi_model/Signal Receiver/ch2_offset', 'Value', sprintf('%d', ChannelOffset(2)));
set_param('multi_model/Signal Receiver/ch3_offset', 'Value', sprintf('%d', ChannelOffset(3)));
set_param('multi_model/Signal Receiver/ch4_offset', 'Value', sprintf('%d', ChannelOffset(4)));
close;

