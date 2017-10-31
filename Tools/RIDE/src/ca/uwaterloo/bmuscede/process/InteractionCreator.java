package ca.uwaterloo.bmuscede.process;

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
import org.apache.poi.xssf.usermodel.IndexedColorMap;
import org.apache.poi.xssf.usermodel.XSSFCellStyle;
import org.apache.poi.xssf.usermodel.XSSFColor;
import org.apache.poi.xssf.usermodel.XSSFFont;
import org.apache.poi.xssf.usermodel.XSSFWorkbook;
import org.apache.poi.xssf.usermodel.extensions.XSSFCellBorder;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTBorder;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTBorderPr;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.CTXf;
import org.openxmlformats.schemas.spreadsheetml.x2006.main.STBorderStyle;

public class InteractionCreator {
	private enum MatrixComponent {LABEL_TITLE, C_LABEL_TITLE, CF_LABEL_TITLE, CF_END_LABEL_TITLE, INTERACTION_DATA_GOOD, 
		INTERACTION_DATA_BAD, INTERACTION_DATA_NEUTRAL, NULL_DATA};
	
	public static final String DIRECT_TEXT = "Direct";
	public static final String INDIRECT_TEXT = "Indirect";
	public static final String NO_COMM_TEXT = "No Communication";
	
	//Matrix Styles
	private Map<MatrixComponent, CellStyle> styles;
	
	private final String MATRIX_HEADER = "Features";
	
	//Specific Rule Sheet Parameters ///////////
	private final int ROW_HEIGHT = 25;
	private final int COLUMN_WIDTH = 15*256;
	private final int VAR_TABLE_START_SIZE = 2;
	////////////////////////////////////////////
	
	private Workbook wb;
	private boolean created;
	
	public InteractionCreator(){		
		//Creates a new workbook.
		wb = new XSSFWorkbook();
		
		//Generates the matrix styles.
		styles = generateMatrixStyles();
		
		//Finally, specifies whether any fields have been created.
		created = false;
	}
	
	public boolean createFeatureMatrixSheet(String sheetName, SortedMap<String, SortedMap<String, String>> data){		
		//Create the sheet.
		Sheet sheet = wb.createSheet(sheetName);
		
        //Parameters for XLS workbooks to ensure compatibility.
        PrintSetup printSetup = sheet.getPrintSetup();
        printSetup.setLandscape(true);
        sheet.setAutobreaks(true);
        printSetup.setFitHeight((short)1);
        printSetup.setFitWidth((short)1);
        
        
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
	
	public boolean addVarColumnBelow(String sheetName, SortedMap<String, Vector<String>> varWrites, int size) {
		//First, we lookup the sheet.
		Sheet sheet = lookupSheet(sheetName);
		if (sheet == null) return false;
		
		//Next, we determine the start row.
		int rowStart = size + VAR_TABLE_START_SIZE;
		
		//Creates the start row.
		Row cRow = sheet.createRow(rowStart);
		cRow.setHeightInPoints(ROW_HEIGHT);
        Cell cTitleCell = cRow.createCell(0);
        cTitleCell.setCellValue("Variables that Callback Functions Write To:");
        sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (rowStart + 1) + ":$J$" + (rowStart + 1)));
        cTitleCell.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
        for (int i = 1; i < 10; i++) cRow.createCell(i).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
        
        //Now, we iterate through and add the variables.
        int curRow = rowStart + 1;
        for (Map.Entry<String, Vector<String>> entry : varWrites.entrySet()){
        	//Create the row.
        	cRow = sheet.createRow(curRow);
        	cRow.setHeightInPoints(ROW_HEIGHT);
        	
        	//Adds the title.
        	Cell title = cRow.createCell(0);
        	title.setCellValue(entry.getKey());
        	sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (curRow + 1) + ":$B$" + (curRow + 1)));
	        title.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        cRow.createCell(1).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
        	
        	//Adds the entry.
        	Cell nData = cRow.createCell(2);
        	nData.setCellValue(entry.getValue().toString());
        	sheet.addMergedRegion(CellRangeAddress.valueOf("$C$" + (curRow + 1) + ":$J$" + (curRow + 1)));
	        nData.setCellStyle(styles.get(MatrixComponent.CF_LABEL_TITLE));
	        for (int j = 3; j < 10; j++) cRow.createCell(j).setCellStyle(styles.get(MatrixComponent.CF_LABEL_TITLE));
	        
        	curRow++;
        }
        
		return true;
	}
	
	public boolean createEmptySheet(String sheetName, String message) {		
		//Create the sheet.
		Sheet sheet = wb.createSheet(sheetName);
		
        //Parameters for XLS workbooks to ensure compatibility.
        PrintSetup printSetup = sheet.getPrintSetup();
        printSetup.setLandscape(true);
        sheet.setAutobreaks(true);
        printSetup.setFitHeight((short)1);
        printSetup.setFitWidth((short)1);
        
        //Creates the row for the 1st section.
        Row itemRow = sheet.createRow(0);
        itemRow.setHeightInPoints(ROW_HEIGHT);
        
        //Creates the header cell.
        sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + 1 + ":$J$" + 1));
        Cell headerCell = itemRow.createCell(0);
        headerCell.setCellValue(message);
        headerCell.setCellStyle(styles.get(MatrixComponent.LABEL_TITLE));
        
        //Merges.
        for (int i = 1; i < 10; i++){
        	itemRow.createCell(i).setCellStyle(styles.get(MatrixComponent.LABEL_TITLE));
        }
        
		created = true;
        return true;
	}
	
	public boolean createMultiWriteSheet(String sheetName, HashMap<String, Vector<String>> results,
			HashMap<String, Integer> outdegreeVals, HashMap<String, String> functionParent) {
		//Creates the sheet.
		Sheet sheet = wb.createSheet(sheetName);
		
        //Parameters for XLS workbooks to ensure compatibility.
        PrintSetup printSetup = sheet.getPrintSetup();
        printSetup.setLandscape(true);
        sheet.setAutobreaks(true);
        printSetup.setFitHeight((short)1);
        printSetup.setFitWidth((short)1);
        
        //Starts by creating the list of publishers.
        Map<String, Integer> hyperlinkPos = new HashMap<String, Integer>();
        int curRow = outdegreeVals.keySet().size() + 2;
        for (Map.Entry<String, Vector<String>> entry : results.entrySet()){
        	Row mainRow = sheet.createRow(curRow);
        	mainRow.setHeightInPoints(ROW_HEIGHT);
            sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (curRow + 1) + ":$J$" + (curRow + 1)));
            Cell headerCell = mainRow.createCell(0);
            headerCell.setCellValue(entry.getKey() + " - ");
            headerCell.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
            curRow++;
            hyperlinkPos.put(entry.getKey(), curRow);
            for (int j = 1; j < 10; j++) mainRow.createCell(j).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
            
            //Now, process the actual cases.
            for (String curPub : entry.getValue()){
            	Row nextRow = sheet.createRow(curRow);
            	nextRow.setHeightInPoints(ROW_HEIGHT);
                sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (curRow + 1) + ":$J$" + (curRow + 1)));
                for (int j = 1; j < 10; j++) nextRow.createCell(j).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
                
                //Adds the data.
                Cell mainCell = nextRow.createCell(0);
                mainCell.setCellValue(curPub);
                mainCell.setCellStyle(styles.get(MatrixComponent.CF_LABEL_TITLE));
            }
            
            curRow += 2;
        }
        
        //Next, creates the lookup table.
		Row titleRow = sheet.createRow(0);
		titleRow.setHeightInPoints(ROW_HEIGHT);
        sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + 1 + ":$M$" + 1));
        for (int j = 1; j < 13; j++) titleRow.createCell(j).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
        Cell headerCell = titleRow.createCell(0);
        headerCell.setCellValue("Number of Publishers that Write to Callback Function in Component:");
        headerCell.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
        
        //Now, we iterate through and add the variables.
        curRow = 1;
        CreationHelper createHelper = wb.getCreationHelper();
        for (Map.Entry<String, Integer> entry : outdegreeVals.entrySet()){
        	//Create the row.
        	Row cRow = sheet.createRow(curRow);
        	cRow.setHeightInPoints(ROW_HEIGHT);
        	
        	//Adds the title.
        	Cell title = cRow.createCell(0);
        	title.setCellValue(entry.getKey());
        	sheet.addMergedRegion(CellRangeAddress.valueOf("$A$" + (curRow + 1) + ":$J$" + (curRow + 1)));
	        title.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        cRow.createCell(1).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        for (int j = 1; j < 10; j++){
	        	cRow.createCell(j).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        }
	        
	        //Creates the hyperlink.
	        Hyperlink link = createHelper.createHyperlink(HyperlinkType.DOCUMENT);
			link.setAddress("\'" + sheetName + "\'" + "!A" + hyperlinkPos.get(entry.getKey()));
    		title.setHyperlink(link);
	        
        	//Adds the entry.
        	Cell nData = cRow.createCell(10);
        	nData.setCellValue(entry.getValue().toString());
        	sheet.addMergedRegion(CellRangeAddress.valueOf("$K$" + (curRow + 1) + ":$M$" + (curRow + 1)));
	        nData.setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        for (int j = 11; j < 13; j++){
	        	cRow.createCell(j).setCellStyle(styles.get(MatrixComponent.C_LABEL_TITLE));
	        }
	        
        	curRow++;
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
	
	private Sheet lookupSheet(String name){
		for (int i = 0; i < wb.getNumberOfSheets(); i++){
			if (wb.getSheetName(i).equals(name)){
				return wb.getSheetAt(i);
			}
		}
		
		return null;
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
        ctitle.setBorderLeft(BorderStyle.MEDIUM);
        ctitle.setBorderRight(BorderStyle.MEDIUM);
        ctitle.setBorderTop(BorderStyle.MEDIUM);
        ctitle.setBorderBottom(BorderStyle.MEDIUM);
		styles.put(MatrixComponent.C_LABEL_TITLE, ctitle);

		CellStyle cftitle = wb.createCellStyle();
        cftitle.setAlignment(HorizontalAlignment.CENTER);
        cftitle.setVerticalAlignment(VerticalAlignment.CENTER);
        cftitle.setBorderLeft(BorderStyle.MEDIUM);
        cftitle.setBorderRight(BorderStyle.MEDIUM);
        cftitle.setBorderTop(BorderStyle.MEDIUM);
        cftitle.setBorderBottom(BorderStyle.MEDIUM);
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
			int idx = stylesSource.putBorder(new XSSFCellBorder(ct, (IndexedColorMap) theme));    
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
