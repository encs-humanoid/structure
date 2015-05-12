package org.ieee.encs.humanoid.humanoidcontrol;

import java.net.URI;
import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.speech.RecognizerIntent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends Activity {

	private AlertDialog.Builder dlgAlert;
	private Handler handler;

	class HumanoidControlNode extends AbstractNodeMain implements MessageListener<Object> {

		private String nodeName;
		private String topic;
		private ConnectedNode connectedNode;
		private Subscriber<Object> subscriber;
		private Publisher<std_msgs.String> listenControlPublisher;
		private Publisher<std_msgs.String> bootPublisher;
		private Publisher<std_msgs.String> sayPublisher;
		private Publisher<std_msgs.String> recognizedSpeechPublisher;

		public HumanoidControlNode(String nodeName, String topic) {
			this.nodeName = nodeName;
			this.topic = topic;
		}

		@Override
		public GraphName getDefaultNodeName() {
			return GraphName.of(nodeName);
		}

		@Override
		public void onStart(final ConnectedNode connectedNode) {
			this.connectedNode = connectedNode;
			this.subscriber = connectedNode.newSubscriber(topic, std_msgs.String._TYPE);
			subscriber.addMessageListener(this);
			this.listenControlPublisher = connectedNode.newPublisher("/listen_control", std_msgs.String._TYPE);
			this.bootPublisher = connectedNode.newPublisher("/boot", std_msgs.String._TYPE);
			this.sayPublisher = connectedNode.newPublisher("/say", std_msgs.String._TYPE);
			this.recognizedSpeechPublisher = connectedNode.newPublisher("/recognized_speech", std_msgs.String._TYPE);
		}

		@Override
		public void onNewMessage(Object object) {
			handler.sendMessage(handler.obtainMessage(0, object));
		}

		public void stopListening() {
			publishListenControl("stop_listening");
		}

		public void resumeListening() {
			publishListenControl("resume_listening");
		}

		private void publishListenControl(String data) {
			std_msgs.String msg = listenControlPublisher.newMessage();
			msg.setData(data);
			listenControlPublisher.publish(msg);
		}

		public void resetAll() {
			publishBoot("reset");
		}

		public void statusAll() {
			publishBoot("status");
		}

		public void stopAll() {
			publishBoot("stop");
		}

		public void shutdownAll() {
			publishBoot("shutdown");
		}

		private void publishBoot(String data) {
			std_msgs.String msg = bootPublisher.newMessage();
			msg.setData(data);
			bootPublisher.publish(msg);
		}

		private void publishSpeech(String say) {
			std_msgs.String msg = sayPublisher.newMessage();
			msg.setData(say);
			sayPublisher.publish(msg);
		}

		public void publishRecognizedText(String recognizedText) {
			std_msgs.String msg = recognizedSpeechPublisher.newMessage();
			msg.setData(recognizedText);
			recognizedSpeechPublisher.publish(msg);
		}
	}

	private static final int REQUEST_CODE = 1234;
	private HumanoidControlNode controlNode;
	private NodeConfiguration nodeConfiguration;
	private NodeMainExecutor executor;
	private boolean retry = true;
	private boolean isRecognizingSpeech = false;

	@Override
	protected void onCreate(Bundle icicle) {
		super.onCreate(icicle);
		dlgAlert = new AlertDialog.Builder(this);
		setContentView(R.layout.main);

		nodeConfiguration = null;
		// while (retry) {
		// try {
		nodeConfiguration = NodeConfiguration.newPublic(Util.getIpAddress(), URI.create("http://192.168.8.100:11311/"));
		// }
		// catch (Exception e) {
		// dlgAlert.setTitle("Error");
		// dlgAlert.setMessage(e.getMessage() + "\n\nTry again?");
		// dlgAlert.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
		// public void onClick(DialogInterface dialog, int buttonId) {
		// retry = true;
		// }
		// });
		// dlgAlert.setNegativeButton("Exit", new DialogInterface.OnClickListener() {
		// public void onClick(DialogInterface dialog, int buttonId) {
		// retry = false;
		// }
		// });
		// dlgAlert.setCancelable(true);
		// dlgAlert.create().show();
		// }
		// }
		//
		// if (nodeConfiguration == null) {
		// this.finish();
		// return;
		// }

		executor = DefaultNodeMainExecutor.newDefault();
		controlNode = newNode();
		executor.execute(controlNode, nodeConfiguration);

		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		Button stopListening = (Button) findViewById(R.id.stop_listening_button);
		stopListening.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				controlNode.stopListening();
			}
		});

		Button resumeListening = (Button) findViewById(R.id.resume_listening_button);
		resumeListening.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				controlNode.resumeListening();
			}
		});

		Button testSpeech = (Button) findViewById(R.id.test_speech_button);
		testSpeech.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				controlNode.publishSpeech("speech node is operational");
			}
		});

		Button talk = (Button) findViewById(R.id.talk_button);
		talk.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				startVoiceRecognitionActivity();
			}
		});

		final Activity activity = this;
		Button reset = (Button) findViewById(R.id.reset_button);
		reset.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				confirm(activity, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						controlNode.resetAll();
					}
				}, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						// Intentionally empty - do nothing
					}
				});
			}
		});

		Button status = (Button) findViewById(R.id.status_button);
		status.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				controlNode.statusAll();
			}
		});

		Button stop = (Button) findViewById(R.id.stop_button);
		stop.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				confirm(activity, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						controlNode.stopAll();
					}
				}, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						// Intentionally empty - do nothing
					}
				});
			}
		});

		Button shutdown = (Button) findViewById(R.id.shutdown_button);
		shutdown.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				confirm(activity, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						controlNode.shutdownAll();
					}
				}, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int buttonId) {
						// Intentionally empty - do nothing
					}
				});
			}
		});

		this.handler = new Handler() {
			@Override
			public void handleMessage(Message msg) {
				std_msgs.String string = (std_msgs.String) msg.obj;

				String data = string.getData();
				TextView sayText = (TextView) findViewById(R.id.sayTextView);
				sayText.setText(data);

				// dlgAlert.setMessage("Message received for " + data);
				// dlgAlert.setTitle("Notification");
				// dlgAlert.setPositiveButton("OK", null);
				// dlgAlert.setCancelable(true);
				// dlgAlert.create().show();

				super.handleMessage(msg);
			}
		};
	}

	public void confirm(Context context, DialogInterface.OnClickListener confirmedListener, DialogInterface.OnClickListener declinedListener) {
		AlertDialog dialog = new AlertDialog.Builder(context).create();
		dialog.setTitle("Confirmation");
		dialog.setMessage("Are you sure you want to proceed?");
		dialog.setCancelable(false);
		dialog.setButton(DialogInterface.BUTTON_POSITIVE, "Yes", confirmedListener);
		dialog.setButton(DialogInterface.BUTTON_NEGATIVE, "No", declinedListener);
		dialog.setIcon(android.R.drawable.ic_dialog_alert);
		dialog.show();
	}

	@Override
	protected void onPause() {
		super.onPause();
		if (!isRecognizingSpeech)
			executor.shutdown();
	}

	@Override
	protected void onResume() {
		super.onResume();
		if (!isRecognizingSpeech) {
			executor = DefaultNodeMainExecutor.newDefault();
			controlNode = newNode();
			executor.execute(controlNode, nodeConfiguration);
		}
		isRecognizingSpeech = false;
	}

	private HumanoidControlNode newNode() {
		return new HumanoidControlNode("humanoid_control_node", "say");
	}

	/**
	 * Fire an intent to start the voice recognition activity.
	 */
	private void startVoiceRecognitionActivity() {
		isRecognizingSpeech = true;
		Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
		intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Humanoid Control...");
		startActivityForResult(intent, REQUEST_CODE);
	}

	/**
	 * Handle the results from the voice recognition activity.
	 */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (requestCode == REQUEST_CODE && resultCode == RESULT_OK) {
			ArrayList<String> matches = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
			if (matches.size() > 0) {
				TextView talkText = (TextView) findViewById(R.id.talkTextView);
				talkText.setText(matches.get(0));
				controlNode.publishRecognizedText(matches.get(0));
			}
		}
		super.onActivityResult(requestCode, resultCode, data);
	}
}
