package ca.uwaterloo.bmuscede.ta;

import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;

public class StringErrorListener extends BaseErrorListener {
	public static final StringErrorListener INSTANCE = new StringErrorListener();

	private String messages = new String();
	
	@Override
	public void syntaxError(Recognizer<?, ?> recognizer, Object offendingSymbol, int line,
							int charPositionInLine, String msg, RecognitionException e) {
		messages += "Line " + line + ":" + charPositionInLine + " " + msg + "\n";
	}
	
	public String getMessages(){
		return messages;
	}
}