   M S B A M L   `   `   `  ����D  @MasterWPF, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null%clr-namespace:MasterWPF	MasterWPF  5   	       MasterWPF.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   @d2http://schemas.microsoft.com/expression/blend/2008     5   	   Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5   	   )localclr-namespace:MasterWPF     5   	     � Title$6  0Server for Multiple Microsoft Kinect (v1 and v2)��5   	   $	��640��6B   $	��480��6O    � 	Resources 5         ClientItemTemplate �� Orientation��%�  &	        �� 5   
   .R�5      �� $ 
Horizontal=�6   .+�5      �� $��32��6   $��32��6$   $��UniformToFill=�6/   $��plug.png��6G   �� 5      .+�5      �� ��6    �� *
ClientIP+�� 5      ��6    �� *
ClientName+�� 5      ��6    �� *
ClientType+�� 5      ��6    �� *ClientStatus+�� 5      ��6    �� *
ClientTime+�� 5      ��6    �� *ClientLatency+5      5      5      5      .��5      �� .+�5   
   �� $��8q�6   .+�5      �� $ 
Horizontal=�6   $��Center=�62   $��8q�6O   .+�5      �� -   5        $��StartNewSessionButton��5      $��Start New Session��68   $��8q�6x   �� 5      -   5        $��SaveSubjectButton��5      $��Save Subject��62   $��8q�6i   {�.  6t   5       5!      �� 5#   
   -   5        $��MainPlaneStackPanel��5#   !   $��16q�6   $y�	Collapsed=�6<   .+�5%      �� -   5         ��Name$ StatusTextBlock��5%      �� 5&      $ 
Horizontal=�6   $��8q�62   .+�5'      �� $
��IP: ��6   $��4q�6(   �� 5(      -   5         ��Name$ IPListCombo��5(   4   $��4q�6I   {�.  6f   �� 5)      $��	Port No: ��6   $��4q�6-   �� 5*      -   5         ��Name$ PortNoTextBox��5*      $��40��61   $
��8688��6?   $��4q�6K   �� 5+      -   5         ��Name$ SelectIPButton��5+      $��Start Server��61   {�.  $��4q�6[   6f   5-      �� 5.      $ 
Horizontal=�6   $��Center=�62   .+�50      �� -   5        $ 
InitButton��50      $��Initiate Clients��6-   $��4q�6H   {�.  6l   �� 51      -	   5        $ TriggerButton��51      $��Start Record��60   $��4q�6G   {�.  6e   �� 52      -
   5        $ PauseButton��52      $��Pause Record��6.   $��4q�6E   {�.  6c   �� 53      -   5        $ 
StopButton��53      $��Stop Record��6-   $��4q�6C   {�.  6`   54      �� 55      -   5         ��Name$ ClientListView��55      #��[  6.    �� SelectionMode$ Single=�6a   $��16q�6x   �� 57      -   5        $��SubjectInfoStackPanel��57      $��8q�67   .+�58      �� $ 
Horizontal=�6   .+�59      �� $��
Subject ID��6    $��8q�62   �� 5:      -   5        $��SubjectIDTextBox��5:      $��8q�66   $��60��6A   �� 5;      -   5        $��IsHealthyCheckBox��5;   E   $��
Is Healthy��6   	 d� 	IsChecked$
	 TrueY�64   $��8q�6^   �� 5<      -   5        $��SubjectConditionTextBox��5<      $��8q�6=   $��60��6H   5=      �� 5>      $ 
Horizontal=�6   .+�5?      �� $
��Age ��6    $��8q�6,   �� 5@      -   5        $��SubjectAgeTextBox��5@      $��8q�67   $��60��6B   �� 5A      $��Gender��6    $��8q�6/   	� 5C      -   5        $��SubjectIsMaleRadioButton��5C   "   $
��Male��6B   $��8q�6Q   $��Center=�6\   	� 5D      -   5        $ ��SubjectIsFemaleRadioButton��5D   "   $��Female��6D   $��8q�6U   $��Center=�6`   5E      �� 5F      $ 
Horizontal=�6   .+�5G      �� $��
Weight(kg)��6    $��4q�62   �� 5H      -   5        $��WeightTextBox��5H      $��4q�63   $��40��6>   �� 5I      $��
Height(cm)��6    $��4q�62   �� 5J      -   5        $��HeightTextBox��5J      $��4q�63   $��40��6>   �� 5K      -   5        $��BMITextBlock��5K       $��4q�64   5L      �� 5M      $ 
Horizontal=�6   $��8q�66   .+�5N      �� $��Dominance :��6    $��8q�63   $��Center=�6>   $��Center=�6Y   �� 5O      $ Vertical=�6!   $��8q�68   .+�5P      �� $
��Hand��6$   $��8q�60   $��Center=�6;   $��Center=�6V   �� 5Q      -   5        $��RightHandCheckBox��5Q   3   $��Right��6#   $��8q�6L   �� 5R      -   5        $��LeftHandCheckBox��5R   2   $
��Left��6#   $��8q�6J   5S      �� 5T      $ Vertical=�6!   $��8q�68   .+�5U      �� $
��Foot��6$   $��8q�60   $��Center=�6;   $��Center=�6V   �� 5V      -   5        $��RightFootCheckBox��5V   3   $��Right��6#   $��8q�6L   �� 5W      -   5        $��LeftFootCheckBox��5W   2   $
��Left��6#   $��8q�6J   5X      �� 5Y      $ Vertical=�6!   $��8q�68   .+�5Z      �� $	��Eye��6$   $��8q�6/   $��Center=�6:   $��Center=�6U   �� 5[      -   5        $��RightEyeCheckBox��5[   3   $��Right��6#   $��8q�6K   �� 5\      -   5        $��LeftEyeCheckBox��5\   2   $
��Left��6#   $��8q�6I   5]      5`      �� 5a      $ 
Horizontal=�6   .+�5b      �� $��	Comments:��6    $��8q�61   �� 5c      -   5        $��CommentsTextBox��5c      $��8q�65   
 �� AcceptsReturn
 . $	��320��6U    �� TextWrapping$
 Wrap=�6a   5d      5f      5h      5j      5k      