package ca.uwaterloo.bmuscede.aide;

import java.awt.Color;
import java.awt.EventQueue;
import java.awt.Font;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.border.EmptyBorder;
import javax.swing.border.LineBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.filechooser.FileNameExtensionFilter;

import ca.uwaterloo.bmuscede.aide.AnalysisGenerator.AnalysisTypes;

public class RIDE extends JFrame {
	static final long serialVersionUID = 395876470924468079L;
	
	private JPanel pnlContent;
	private JTextField txtLoadFileName;
	private JTextField txtSaveFileName;
	private JLabel lblNOutput;
	private JButton btnSave;
	private JLabel lblInfoTitle;
	private JLabel lblInfo;
	private JRadioButton rdoOutput;
	private JRadioButton rdoNoOutput;
	@SuppressWarnings("rawtypes")
	private JList lstOp;
	
	private boolean changed = false;
	private Set<Integer> prevItems = new HashSet<Integer>();
	
	private final String DEFAULT_DESC = "<html><center>Information about each analysis "
			+ "type will be shown here when selected...</center></html>";
	private final String PROG_TITLE = "ROS Interaction Detector (RIDE)";


	
	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					RIDE frame = new RIDE();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the frame.
	 * @throws SecurityException 
	 * @throws NoSuchFieldException 
	 * @throws IllegalAccessException 
	 * @throws IllegalArgumentException 
	 */
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public RIDE() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {		
		setIconImage(Toolkit.getDefaultToolkit().getImage(RIDE.class.getResource("/ca/uwaterloo/bmuscede/aide/aide.png")));
		setResizable(false);
		setTitle(PROG_TITLE);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 524, 415);
		pnlContent = new JPanel();
		pnlContent.setBorder(new EmptyBorder(5, 5, 5, 5));
		setContentPane(pnlContent);
		pnlContent.setLayout(null);
		
		//Ubuntu title workaround.
		Toolkit xToolkit = Toolkit.getDefaultToolkit();
		java.lang.reflect.Field awtAppClassNameField;
		awtAppClassNameField = xToolkit.getClass().getDeclaredField("awtAppClassName");
		awtAppClassNameField.setAccessible(true);
		awtAppClassNameField.set(xToolkit, PROG_TITLE);
		  
		JPanel pnlStep1 = new JPanel();
		pnlStep1.setBorder(new TitledBorder(null, "Load Tuple-Attribute File", TitledBorder.LEADING, TitledBorder.TOP, null, null));
		pnlStep1.setBounds(12, 12, 500, 65);
		pnlContent.add(pnlStep1);
		pnlStep1.setLayout(null);
		
		txtLoadFileName = new JTextField();
		txtLoadFileName.setEditable(false);
		txtLoadFileName.setBounds(12, 27, 347, 19);
		pnlStep1.add(txtLoadFileName);
		txtLoadFileName.setColumns(10);
		
		JButton btnLoad = new JButton("Load...");
		btnLoad.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				loadTAFile();
			}
		});
		btnLoad.setBounds(371, 24, 117, 25);
		pnlStep1.add(btnLoad);
		
		JPanel pnlStep2 = new JPanel();
		pnlStep2.setLayout(null);
		pnlStep2.setBorder(new TitledBorder(new LineBorder(new Color(184, 207, 229)), "Select Analysis Operations", 
				TitledBorder.LEADING, TitledBorder.TOP, null, new Color(51, 51, 51)));
		pnlStep2.setBounds(12, 89, 500, 141);
		pnlContent.add(pnlStep2);
		
		JScrollPane scrOp = new JScrollPane();
		scrOp.setBounds(12, 26, 235, 103);
		pnlStep2.add(scrOp);
		
		lstOp = new JList(AnalysisGenerator.AnalysisTypes.values());
		lstOp.addListSelectionListener(new ListSelectionListener() {
			public void valueChanged(ListSelectionEvent e) {
				//Check if we have an empty selection.
				if (lstOp.getSelectedIndices().length == 0){
					lblInfoTitle.setText("Select Operations!");
					lblInfo.setText(DEFAULT_DESC);
					return;
				}
				
				//Get the selected items.
				List<Integer> curSel = new ArrayList<Integer>();
				for (int item : lstOp.getSelectedIndices()){
					curSel.add(item);
				}
				Set<Integer> currentItems = new HashSet<Integer>(curSel);
				
				//Performs set difference.
				currentItems.removeAll(prevItems);
				if (currentItems.size() == 1){
					Integer curInd = currentItems.iterator().next();
					
					//Updates the description system.
					AnalysisTypes type = (AnalysisTypes) lstOp.getModel().getElementAt(curInd);
					lblInfoTitle.setText(type.toString());
					lblInfo.setText(type.getDescription());
				}
				
				//Updates the previous items.
				prevItems = new HashSet<Integer>(curSel);
			}
		});
		scrOp.setViewportView(lstOp);
		
		lblInfoTitle = new JLabel("Select Operations!");
		lblInfoTitle.setHorizontalAlignment(SwingConstants.CENTER);
		lblInfoTitle.setFont(new Font("Dialog", Font.BOLD, 12));
		lblInfoTitle.setBounds(257, 22, 231, 26);
		pnlStep2.add(lblInfoTitle);
		
		lblInfo = new JLabel(DEFAULT_DESC);
		lblInfo.setFont(new Font("Dialog", Font.PLAIN, 12));
		lblInfo.setBounds(257, 38, 231, 91);
		pnlStep2.add(lblInfo);
		
		JPanel pnlStep3 = new JPanel();
		pnlStep3.setLayout(null);
		pnlStep3.setBorder(new TitledBorder(new LineBorder(new Color(184, 207, 229)), "Select Output File", TitledBorder.LEADING, TitledBorder.TOP, null, new Color(51, 51, 51)));
		pnlStep3.setBounds(12, 242, 500, 92);
		pnlContent.add(pnlStep3);
		
		lblNOutput = new JLabel("No File Output Selected!");
		lblNOutput.setVisible(false);
		lblNOutput.setHorizontalAlignment(SwingConstants.CENTER);
		lblNOutput.setFont(new Font("Dialog", Font.BOLD | Font.ITALIC, 16));
		lblNOutput.setBounds(12, 60, 488, 15);
		pnlStep3.add(lblNOutput);
		
		txtSaveFileName = new JTextField();
		txtSaveFileName.setEditable(false);
		txtSaveFileName.setColumns(10);
		txtSaveFileName.setBounds(12, 58, 347, 19);
		pnlStep3.add(txtSaveFileName);
		
		btnSave = new JButton("Save...");
		btnSave.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				saveFile();
			}
		});
		btnSave.setBounds(371, 55, 117, 25);
		pnlStep3.add(btnSave);
		
		rdoOutput = new JRadioButton("Output Results");
		rdoOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				configureSaveBox(true);
			}
		});
		rdoOutput.setSelected(true);
		rdoOutput.setBounds(12, 22, 167, 23);
		pnlStep3.add(rdoOutput);
		
		rdoNoOutput = new JRadioButton("No Results Output\n");
		rdoNoOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				configureSaveBox(false);
			}
		});
		rdoNoOutput.setBounds(183, 22, 167, 23);
		pnlStep3.add(rdoNoOutput);
		
		JButton btnClose = new JButton("Close");
		btnClose.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (!changed) {
					dispose();
					return;
				}
				
				//Check if the user would like to save.
				int result = JOptionPane.showConfirmDialog(null, "Changes have been made! Are you sure you want to exit?",
						"Autonomouse Interaction Detector", JOptionPane.YES_NO_OPTION);
				if (result == JOptionPane.YES_OPTION){
					dispose();
				}
			}
		});
		btnClose.setBounds(12, 346, 117, 25);
		pnlContent.add(btnClose);
		
		JButton btnRun = new JButton("Run!");
		btnRun.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				boolean status = verify();
				if (status) {
					AnalysisGenerator gen = new AnalysisGenerator(lstOp.getSelectedValuesList());
					
					//Runs analysis.
					boolean result = true;
					if (rdoNoOutput.isSelected()){
						result = gen.runAnalysis(txtLoadFileName.getText());
					} else {
						result = gen.runAnalysis(txtLoadFileName.getText(), txtSaveFileName.getText());
					}
					
					//Prints a status message to the user.
					if (result){
						JOptionPane.showMessageDialog(null, "All operations completed successfully!", "Autonomous Interaction Detector",
								JOptionPane.INFORMATION_MESSAGE);
					} else {
						String errText = gen.getErrorMessage();
						JOptionPane.showMessageDialog(null, "There were errors encountered while processing:\n" + errText, 
								"Autonomous Interaction Detector", JOptionPane.ERROR_MESSAGE);
					}
				}
			}
		});
		btnRun.setBounds(395, 346, 117, 25);
		pnlContent.add(btnRun);
		
		//Sets up radio button groups.
		ButtonGroup btnGroup = new ButtonGroup();
		btnGroup.add(rdoOutput);
		btnGroup.add(rdoNoOutput);
	}
	
	private void loadTAFile(){	
		//Opens up an import dialog.
		JFileChooser fileChooser = new JFileChooser();
		fileChooser.setCurrentDirectory(new File(System.getProperty("user.home")));
						
		FileNameExtensionFilter filter = new FileNameExtensionFilter("TA Files", "ta");
		fileChooser.setFileFilter(filter);
		fileChooser.setDialogTitle("Load TA File...");
				
		//Reads what was selected.
		int result = fileChooser.showOpenDialog(null);
		
		//Loads in the data.
		if (result == JFileChooser.APPROVE_OPTION) {
			changed = true;
			
			//Get the filename.
			String filename = fileChooser.getSelectedFile().getAbsolutePath();
			txtLoadFileName.setText(filename);
		}
	}
	
	private void saveFile(){
        //Configures the save dialog.
	    JFileChooser fileChooser = new JFileChooser(); 
		fileChooser.setCurrentDirectory(new File(System.getProperty("user.home")));
	    fileChooser.setDialogTitle("Excel File to Save Results...");
		FileNameExtensionFilter filter = new FileNameExtensionFilter("Excel Files", "xlsx");
		fileChooser.setFileFilter(filter);

	    //Now, opens the dialog and checks the result.
	    if (fileChooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
	    	File selFile = fileChooser.getSelectedFile();
	    	String fileString = selFile.getAbsolutePath();
	    	if (!filter.accept(selFile)){
	    		fileString += ".xlsx";
	    	}
	    	changed = true;
	    	
	    	txtSaveFileName.setText(fileString);
	      }
	}
	
	private void configureSaveBox(boolean outputEnabled){
		if (outputEnabled){
			lblNOutput.setVisible(false);
			txtSaveFileName.setVisible(true);
			btnSave.setVisible(true);
		} else {
			lblNOutput.setVisible(true);
			txtSaveFileName.setVisible(false);
			btnSave.setVisible(false);
			txtSaveFileName.setText("");
		}
	}
	
	private boolean verify() {
		boolean status = true;
		String errorMsg = "";
		
		if (txtSaveFileName.getText().equals("") && rdoOutput.isSelected()){
			status = false;
			errorMsg += "- You must either select an output directory or choose not to output the results!";
		}
		if (lstOp.getSelectedIndices().length == 0){
			status = false;
			if (!errorMsg.equals("")) errorMsg += "\n";
			errorMsg += "- You must select at least one operator to perform!";
		}
		if (txtLoadFileName.getText().equals("")){
			status = false;
			if (!errorMsg.equals("")) errorMsg += "\n";
			errorMsg += "- You must select an input TA file to load from!";
		}
		
		//Finally, prints an error message.
		if (!status){
			JOptionPane.showMessageDialog(null, "Errors must be corrected before proceeding:\n" + errorMsg, 
					"Autonomous Interaction Detector", JOptionPane.ERROR_MESSAGE);
		}
		
		return status;
	}
}
