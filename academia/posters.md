# How to make a poster

1. Make the poster using PowerPoint, Keynote, or Google Slides. A standard portrait size is 32in wide by 40in tall, but different conferences might request that you print your poster in a different size.
2. Set up printer access in CSE according to the [Printing support page](https://www.cs.washington.edu/lab/printing)
3. Save your poster as a PDF
4. Run `pdf2ps poster.pdf poster.ps` to convert your poster to raw PostScript.
5. Send the PostScript to the poster printer: `lpr -P psclarge poster.ps`
6. If the printer is out of paper, replace the roll according to the instruction manual on the printer
7. Trim your poster as needed using the paper cutter
8. If you want to attach your poster to a foam board, then find an unused piece of foam board in the poster room. Spray it with the special spray in the room, then carefully put your poster down on top.

This workflow has worked for lab members in the past. CSE support has a page on [poster printing](https://www.cs.washington.edu/lab/printing/poster-printer).

## Templates and examples
- [Google Slides](https://docs.google.com/presentation/d/1zlhNrCor8dvBUCoq2JXy_o81-On_C_tk56Hf9TPVCvQ/edit?pli=1#slide=id.p13). Open it while signed into your Google account, then select File->Make a copy
- [Examples from the HCR lab](https://drive.google.com/folderview?id=0B5wyq66plcULZWFTRmpWLXJEYU0&usp=drive_web)
- More templates available on the [CSE support page](https://www.cs.washington.edu/lab/printing/poster-printer)

## Advanced
### A0 printing with a non-A0 PDF
If you are trying to print an A0 poster, but the PDF is not size A0, try printing the poster directly from a PDF viewer. Set the paper size to A0 and then say "scale content to fit page."

You can also try resizing the poster to A0 with:
```
pdftops -paperw 2384 -paperh 3370 -expand poster.pdf poster.ps
```
