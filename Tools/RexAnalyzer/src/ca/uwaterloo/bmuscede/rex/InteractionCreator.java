package ca.uwaterloo.bmuscede.rex;

import java.io.FileOutputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.SortedMap;
import java.util.Vector;

import org.apache.poi.common.usermodel.HyperlinkType;
import org.apache.poi.ss.usermodel.BorderStyle;
import org.apache.poi.ss.usermodel.Cell;
import org.apache.poi.ss.usermodel.CellStyle;
import org.apache.poi.ss.usermodel.CreationHelper;
import org.apache.poi.ss.usermodel.FillPatternType;
import org.apache.poi.ss.usermodel.Font;
import org.apache.poi.ss.usermodel.HorizontalAlignment;
import org.apache.poi.ss.usermodel.Hyperlink;
import org.apache.poi.ss.usermodel.IndexedColors;
import org.apache.poi.ss.usermodel.PrintSetup;
import org.apache.poi.ss.usermodel.Row;
import org.apache.poi.ss.usermodel.Sheet;
import org.apache.poi.ss.usermodel.VerticalAlignment;
import org.apache.poi.ss.usermodel.Workbook;
import org.apache.poi.ss.util.CellRangeAddress;
import org.apache.poi.xssf.model.StylesTable;
import org.apache.poi.xssf.model.ThemesTable;
import org.apache.poi.xssf.usermodel.XSSFCellStyle;
import org.apache.poi.xssf.usermodel.XSSFColor;
import org.apache.poi.xssf.usermodel.XSSFFont;
import org.apache.poi.xssf.usermodel.XSSFWorkbook;
import org.apache.poi.xssf.usermodel.extensions.XSSFCellBorder;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTBorder;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTBorderPr;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTXf;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.STBorderStyle;

import ca.uwaterloo.bmuscede.rex.PathData.PathType;

public class InteractionCreator {
	private enum MatrixComponent {LABEL_TITLE, C_LABEL_TITLE, CF_LABEL_TITLE, CF_END_LABEL_TITLE, INTERACTION_DATA_GOOD, 
		INTERACTION_DATA_BAD, INTERACTION_DATA_NEUTRAL, NULL_DATA};
	
	private final String DIRECT_TEXT = "Direct";
	private final String INDIRECT_TEXT = "Indirect";
	private final String NO_COMM_TEXT = "No Communication";
	
	private final String MATRIX_NAME = "Feature Interaction Matrix";
	private final String MATRIX_HEADER = "Features";
	
	private Workbook wb;
	private boolean created;
	
	public InteractionCreator(){		
		//Creates a new workbook.
		wb = new XSSFWorkbook();
		
		//Finally, specifies whether any fields have been created.
		created = false;
	}
	
	public boolean createFeatureMatrixSheet(SortedMap<String, SortedMap<String, String>> data, Vector<PathData> cases){
		//Specific Rule Sheet Parameters ///////////
		final int ROW_HEIGHT = 25;
		final int COLUMN_WIDTH = 15*256;
		final int C_POS = 2;
		////////////////////////////////////////////
		
		//Generates the matrix styles.
		Map<MatrixComponent, CellStyle> styles = generateMatrixStyles();
		
		//Create the sheet.
		Sheet sheet = wb.createSheet(MATRIX_NAME);
		
        //Parameters for XLS workbooks to ensure compatibility.
        PrintSetup printSetup = sheet.getPrintSetup();
        printSetup.setLandscape(true);
        sheet.setAutobreaks(true);
        printSetup.setFitHeight((short)1);
        printSetup.setFitWidth((short)1);
        
        //First, generates a list of counterexamples.
        if (cases != null){
	        int cRowStart = data.size() + C_POS;
	        Row cRow = sheet.createRow(cRowStart);
	        cRow.setHeightInPoints(ROW_HEIGHT);
	        Cell cTitleCell = cRow.createCell(0);
	        cTitleCell.setCellValue("Interaction Rationale:");
	        sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (cRowStart + 1) + ":$J$" + (cRowStart + 1)));
	        cTitleCell.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        for (int i = 1; i < 10; i++){
	        	cRow.createCell(i).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        }
	        
	        //Next, generates the children.
	        int i = 1;
	        for (PathData curCase : cases){
	        	if (curCase.getType() != PathType.INDIRECT) continue;
	        	
	        	Row caseRow = sheet.createRow(cRowStart + i);
	        	caseRow.setHeightInPoints(ROW_HEIGHT);
	        	
	        	//Adds the title.
	        	Cell title = caseRow.createCell(0);
	        	title.setCellValue(curCase.getSrc() + " to " + curCase.getDst());
	        	sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (cRowStart + 1 + i) + ":$B$" + (cRowStart + 1 + i)));
		        title.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
		        caseRow.createCell(1).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
		        
	        	//Adds the entry.
	        	Cell nData = caseRow.createCell(2);
	        	nData.setCellValue(curCase.getPath().toString());
	        	sheet.addMergedRegion(CellRangeAddress.valueOf("$C$" + (cRowStart + 1 + i) + ":$J$" + (cRowStart + 1 + i)));
		        nData.setCellStyle(styles.get(MatrixComponent.CF_LABEL_TITLE));
		        for (int j = 3; j < 10; j++){
		        	if (j + 1 == 10) caseRow.createCell(j).setCellStyle(styles.get(MatrixComponent.CF_END_LABEL_TITLE));
		        	else caseRow.createCell(j).setCellStyle(styles.get(MatrixComponent.CF_LABEL_TITLE));
		        }
		        
	        	i++;
	        }
        }
        
        //Creates the row for the 1st section.
        Row itemRow = sheet.createRow(0);
        itemRow.setHeightInPoints(ROW_HEIGHT);
        
        //Creates the header cell.
        Cell headerCell = itemRow.createCell(0);
        headerCell.setCellValue(MATRIX_HEADER);
        headerCell.setCellStyle(styles.get(MatrixComponent.LABEL_TITLE));

        //Now we iterate through and generate the labels for the header.
        Cell current;
        int i = 1;
        for (String label : data.keySet()){
        	current = itemRow.createCell(i);
        	current.setCellValue(label);
        	current.setCellStyle(styles.get(MatrixComponent.LABEL_TITLE));
        	
        	i++;
        }
        
        //Now fills in the rest of the matrix.
        CreationHelper createHelper = wb.getCreationHelper();
        int rowIndex = 1;
        for (String label : data.keySet()){
        	Row currentRow = sheet.createRow(rowIndex);
        	currentRow.setHeightInPoints(ROW_HEIGHT);
        
        	//Creates the Y axis for the matrix for the current row.
        	Cell currentCell = currentRow.createCell(0);
        	currentCell.setCellValue(label);
        	currentCell.setCellStyle(styles.get(MatrixComponent.LABEL_TITLE));
        	
        	//Now, auto fills in the rest of the sheet.
        	int columnIndex = 1;
        	SortedMap<String, String> rowData = data.get(label);
        	for (String intData : rowData.keySet()){
        		currentCell = currentRow.createCell(columnIndex);
        		if (rowIndex == columnIndex){
        			currentCell.setCellStyle(styles.get(MatrixComponent.NULL_DATA));
        		} else {
        			String curCellData = rowData.get(intData);
            		currentCell.setCellValue(curCellData);
            		
            		//Determines which style to apply.
            		if (curCellData.equals(DIRECT_TEXT)){
                		currentCell.setCellStyle(styles.get(MatrixComponent.INTERACTION_DATA_BAD));	
            		} else if (curCellData.equals(INDIRECT_TEXT)){
                		currentCell.setCellStyle(styles.get(MatrixComponent.INTERACTION_DATA_NEUTRAL));
                		
                    	//Generates a hyperlink.
                		if (cases != null){
                			Hyperlink link = createHelper.createHyperlink(HyperlinkType.DOCUMENT);
                			
                			boolean loop = true;
                			int pos = data.size() + C_POS + 1;
                			while (loop){
                				Row curRow = sheet.getRow(pos);
                				if (curRow == null) {
                					loop = false;
                					pos = -1;
                					continue;
                				}
                				
                				String cData = curRow.getCell(0).getStringCellValue();
                				if (cData.equals(label + " influences " + intData)){
                					loop = false;
                				} else {
                					pos++;
                				}
                			}
                			
                			if (pos == -1) 
                				continue;
                			
                			link.setAddress("\'" + MATRIX_NAME + "\'" + "!A" + (pos + 1));
                    		currentCell.setHyperlink(link);
                		}
            		} else if (curCellData.equals(NO_COMM_TEXT)){
                		currentCell.setCellStyle(styles.get(MatrixComponent.INTERACTION_DATA_GOOD));
            		} else {
                		currentCell.setCellStyle(styles.get(MatrixComponent.NULL_DATA));
            		}
        		}
        		
        		columnIndex++;
        	}
        	
        	rowIndex++;
        }
        
        //Finally, sets column widths.
        for (int count = 0; count < rowIndex; count++){
        	sheet.setColumnWidth(count, COLUMN_WIDTH);
        }
        
        created = true;
		return true;
	}
	
	public boolean outputWorkbook(String fileName){
		//Aborts immediately if a blank workbook is supplied.
		if (created == false) return false;
		
        //Checks if a file ends in the extensions.
		if (fileName.endsWith(".xlsx")){
        	fileName = fileName.substring(0, fileName.length() - 5);
        }
        
        //Finally, writes out.
        fileName += ".xlsx";
        FileOutputStream out;
		try {
			out = new FileOutputStream(fileName);
			
	        wb.write(out);
	        out.close();
	        wb.close();
		} catch (Exception e) {
			return false;
		}

        return true;
	}
	
	private Map<MatrixComponent, CellStyle> generateMatrixStyles(){
		Map<MatrixComponent, CellStyle> styles = new HashMap<MatrixComponent, CellStyle>();
		
		//Create the TITLE cell style.
		CellStyle title = wb.createCellStyle();
		Font titleFont = wb.createFont();
		titleFont.setBold(true);
        title.setAlignment(HorizontalAlignment.CENTER);
        title.setVerticalAlignment(VerticalAlignment.CENTER);
        title.setFont(titleFont);
        title.setBorderLeft(BorderStyle.MEDIUM);
        title.setBorderRight(BorderStyle.MEDIUM);
        title.setBorderTop(BorderStyle.MEDIUM);
        title.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.LABEL_TITLE, title);
		
		//Create the condLabel
		CellStyle ctitle = wb.createCellStyle();
        ctitle.setAlignment(HorizontalAlignment.CENTER);
        ctitle.setVerticalAlignment(VerticalAlignment.CENTER);
        ctitle.setFont(titleFont);
        title.setBorderLeft(BorderStyle.MEDIUM);
        title.setBorderRight(BorderStyle.MEDIUM);
        title.setBorderTop(BorderStyle.MEDIUM);
        title.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.C_LABEL_TITLE, title);

		CellStyle cftitle = wb.createCellStyle();
        ctitle.setAlignment(HorizontalAlignment.CENTER);
        ctitle.setVerticalAlignment(VerticalAlignment.CENTER);
        ctitle.setFont(titleFont);
        title.setBorderTop(BorderStyle.MEDIUM);
        title.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.CF_LABEL_TITLE, cftitle);
		
		CellStyle cfendtitle = wb.createCellStyle();
        ctitle.setAlignment(HorizontalAlignment.CENTER);
        ctitle.setVerticalAlignment(VerticalAlignment.CENTER);
        title.setBorderRight(BorderStyle.MEDIUM);
        title.setBorderTop(BorderStyle.MEDIUM);
        title.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.CF_END_LABEL_TITLE, cfendtitle);
		
		//Create the INTERACTION_DATA style.
		XSSFCellStyle good = (XSSFCellStyle) wb.createCellStyle();
        good.setAlignment(HorizontalAlignment.CENTER);
        good.setVerticalAlignment(VerticalAlignment.CENTER);
		XSSFFont goodFont = (XSSFFont) wb.createFont();
		goodFont.setItalic(true);
		XSSFColor goodColor = new XSSFColor(new java.awt.Color(0, 97, 0));
		goodFont.setColor(goodColor);
        good.setFont(goodFont);
        XSSFColor gbColor = new XSSFColor(new java.awt.Color(198, 239, 206));
        good.setFillForegroundColor(gbColor);
        good.setFillPattern(FillPatternType.SOLID_FOREGROUND);
        good.setBorderLeft(BorderStyle.MEDIUM);
        good.setBorderRight(BorderStyle.MEDIUM);
        good.setBorderTop(BorderStyle.MEDIUM);
        good.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.INTERACTION_DATA_GOOD, good);
		
		XSSFCellStyle bad = (XSSFCellStyle) wb.createCellStyle();
        bad.setAlignment(HorizontalAlignment.CENTER);
        bad.setVerticalAlignment(VerticalAlignment.CENTER);
		XSSFFont badFont = (XSSFFont) wb.createFont();
		badFont.setItalic(true);
		XSSFColor badColor = new XSSFColor(new java.awt.Color(156, 0, 6));
		badFont.setColor(badColor);
        bad.setFont(badFont);
        XSSFColor bbColor = new XSSFColor(new java.awt.Color(255, 199, 206));
        bad.setFillForegroundColor(bbColor);
        bad.setFillPattern(FillPatternType.SOLID_FOREGROUND);
        bad.setBorderLeft(BorderStyle.MEDIUM);
        bad.setBorderRight(BorderStyle.MEDIUM);
        bad.setBorderTop(BorderStyle.MEDIUM);
        bad.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.INTERACTION_DATA_BAD, bad);
		
		XSSFCellStyle neutral = (XSSFCellStyle) wb.createCellStyle();
        neutral.setAlignment(HorizontalAlignment.CENTER);
        neutral.setVerticalAlignment(VerticalAlignment.CENTER);
		XSSFFont neutralFont = (XSSFFont) wb.createFont();
		neutralFont.setItalic(true);
		XSSFColor neutralColor = new XSSFColor(new java.awt.Color(156, 101, 0));
		neutralFont.setColor(neutralColor);
        neutral.setFont(neutralFont);
        XSSFColor nbColor = new XSSFColor(new java.awt.Color(255, 235, 156));
        neutral.setFillForegroundColor(nbColor);
        neutral.setFillPattern(FillPatternType.SOLID_FOREGROUND);
        neutral.setBorderLeft(BorderStyle.MEDIUM);
        neutral.setBorderRight(BorderStyle.MEDIUM);
        neutral.setBorderTop(BorderStyle.MEDIUM);
        neutral.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.INTERACTION_DATA_NEUTRAL, neutral);
		
		//Create the NULL_DATA style.
		CellStyle nullStyle = wb.createCellStyle();
        nullStyle.setFillForegroundColor(IndexedColors.GREY_25_PERCENT.getIndex());
        nullStyle.setFillPattern(FillPatternType.SOLID_FOREGROUND);
        nullStyle.setBorderLeft(BorderStyle.MEDIUM);
        nullStyle.setBorderRight(BorderStyle.MEDIUM);
        nullStyle.setBorderTop(BorderStyle.MEDIUM);
        nullStyle.setBorderBottom(BorderStyle.MEDIUM);
        	
        //Get data necessary to generate the diagonal borders.
    	StylesTable stylesSource = ((XSSFWorkbook) wb).getStylesSource();
    	ThemesTable theme = stylesSource.getTheme();
    	CTXf cellXf = ((XSSFCellStyle) nullStyle).getCoreXf();
    	setBorderDiagonal(BorderStyle.THIN, stylesSource, cellXf, theme);
    	
		styles.put(MatrixComponent.NULL_DATA, nullStyle);
		
		return styles;
	}
	
	private void setBorderDiagonal(BorderStyle border, StylesTable stylesSource, CTXf cellXf , ThemesTable theme){
		//Gets information about the border we set.  
		CTBorder ct = getCTBorder(stylesSource, cellXf);
		CTBorderPr pr = ct.isSetDiagonal() ? ct.getDiagonal() : ct.addNewDiagonal();
		
		//Sees what we want to do with the border.
		if(border == BorderStyle.NONE) {
			ct.unsetDiagonal();
		} else {
			//Sets the borders
			ct.setDiagonalDown(true);
			ct.setDiagonalUp(true);
			pr.setStyle(STBorderStyle.Enum.forInt(border.getCode() + 1));
		}
			int idx = stylesSource.putBorder(new XSSFCellBorder(ct, theme));    
			cellXf.setBorderId(idx);
			cellXf.setApplyBorder(true); 
	}
	
	 private static CTBorder getCTBorder(StylesTable stylesSource, CTXf cellXf ){
		 CTBorder ct;
	 
		 if(cellXf.getApplyBorder()) {
			 int idx = (int) cellXf.getBorderId();
			 XSSFCellBorder cf =stylesSource.getBorderAt(idx);
			 ct = (CTBorder)cf.getCTBorder().copy();
		 } else {
			 ct = CTBorder.Factory.newInstance();
		 }
		 
		 return ct;
	 }
}
