package org.ieee.encs.humanoid.humanoidcontrol;

import java.net.URI;
import java.util.ArrayList;

import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.PublisherListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.speech.RecognizerIntent;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends Activity {

	private AlertDialog.Builder dlgAlert;
	private Handler handler;
    private Button stopListeningButton;
    private Button resumeListeningButton;
    private Button testSpeechButton;
    private Button talkButton;
    private Button resetButton;
    private Button statusButton;
    private Button stopButton;
    private Button shutdownButton;

    enum ConnectionState {
        DISCONNECTED, CONNECTING, CONNECTED
    }

    interface UIUpdate {
        public void update();
    }

	class HumanoidControlNode extends AbstractNodeMain implements MessageListener<Object> {
        final String TAG = "HumanoidControlNode";

        class MyPublisherListener implements PublisherListener<std_msgs.String> {
            private Button[] buttons;

            public MyPublisherListener(Button... buttons) {
                this.buttons = buttons;
            }

            @Override
            public void onNewSubscriber(Publisher<std_msgs.String> publisher, SubscriberIdentifier subscriberIdentifier) {
                Log.v(TAG, "MyPublisherListener.onNewSubscriber called for " + publisher + " and " + subscriberIdentifier);
                for (final Button button : buttons) {
                    onNewMessage(new UIUpdate() {
                        public void update() {
                            button.setEnabled(true);
                        }
                    });
                }
            }

            @Override
            public void onShutdown(Publisher<std_msgs.String> publisher) {
                Log.v(TAG, "MyPublisherListener.onShutdown called for " + publisher);
            }

            @Override
            public void onMasterRegistrationSuccess(Publisher<std_msgs.String> publisher) {
                Log.v(TAG, "MyPublisherListener.onMasterRegistrationSuccess called for " + publisher);
            }

            @Override
            public void onMasterRegistrationFailure(Publisher<std_msgs.String> publisher) {
                Log.e(TAG, "MyPublisherListener.onMasterRegistrationFailure called for " + publisher);
            }

            @Override
            public void onMasterUnregistrationSuccess(Publisher<std_msgs.String> publisher) {
                Log.v(TAG, "MyPublisherListener.onMasterUnregistrationSuccess called for " + publisher);
            }

            @Override
            public void onMasterUnregistrationFailure(Publisher<std_msgs.String> publisher) {
                Log.e(TAG, "MyPublisherListener.onMasterUnregistrationFailure called for " + publisher);
            }
        }

        class MySubscriberListener implements SubscriberListener<Object> {

            @Override
            public void onNewPublisher(Subscriber<Object> objectSubscriber, PublisherIdentifier publisherIdentifier) {
                Log.v(TAG, "MySubscriberListener.onNewPublisher called for " + objectSubscriber + " and " + publisherIdentifier);
            }

            @Override
            public void onShutdown(Subscriber<Object> objectSubscriber) {
                Log.v(TAG, "MySubscriberListener.onShutdown called for " + objectSubscriber);
            }

            @Override
            public void onMasterRegistrationSuccess(Subscriber<Object> objectSubscriber) {
                Log.v(TAG, "MySubscriberListener.onMasterRegistrationSuccess called for " + objectSubscriber);
            }

            @Override
            public void onMasterRegistrationFailure(Subscriber<Object> objectSubscriber) {
                Log.v(TAG, "MySubscriberListener.onMasterRegistrationFailure called for " + objectSubscriber);
            }

            @Override
            public void onMasterUnregistrationSuccess(Subscriber<Object> objectSubscriber) {
                Log.v(TAG, "MySubscriberListener.onMasterUnregistrationSuccess called for " + objectSubscriber);
            }

            @Override
            public void onMasterUnregistrationFailure(Subscriber<Object> objectSubscriber) {
                Log.v(TAG, "MySubscriberListener.onMasterUnregistrationFailure called for " + objectSubscriber);
            }
        }

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
			this.subscriber.addMessageListener(this);
            this.subscriber.addSubscriberListener(new MySubscriberListener());
			this.listenControlPublisher = connectedNode.newPublisher("/listen_control", std_msgs.String._TYPE);
            this.listenControlPublisher.addListener(new MyPublisherListener(stopListeningButton, resumeListeningButton));
			this.bootPublisher = connectedNode.newPublisher("/boot", std_msgs.String._TYPE);
            this.bootPublisher.addListener(new MyPublisherListener(resetButton, statusButton, stopButton, shutdownButton));
			this.sayPublisher = connectedNode.newPublisher("/say", std_msgs.String._TYPE);
            this.sayPublisher.addListener(new MyPublisherListener(testSpeechButton));
			this.recognizedSpeechPublisher = connectedNode.newPublisher("/recognized_speech", std_msgs.String._TYPE);
            this.recognizedSpeechPublisher.addListener(new MyPublisherListener(talkButton));
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

		private void publishListenControl(java.lang.String data) {
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
    private ConnectionState connectionState = ConnectionState.DISCONNECTED;
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

		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        final Button connectButton = (Button) findViewById(R.id.connectButton);
        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                connectDisconnectNode(v);
            }
        });

        stopListeningButton = (Button) findViewById(R.id.stop_listening_button);
		stopListeningButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (controlNode != null)
                    controlNode.stopListening();
            }
        });

        resumeListeningButton = (Button) findViewById(R.id.resume_listening_button);
		resumeListeningButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (controlNode != null)
                    controlNode.resumeListening();
            }
        });

        testSpeechButton = (Button) findViewById(R.id.test_speech_button);
		testSpeechButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (controlNode != null)
                    controlNode.publishSpeech("speech node is operational");
            }
        });

        talkButton = (Button) findViewById(R.id.talk_button);
		talkButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startVoiceRecognitionActivity();
            }
        });

		final Activity activity = this;
        resetButton = (Button) findViewById(R.id.reset_button);
		resetButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                confirm(activity, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        if (controlNode != null)
                            controlNode.resetAll();
                    }
                }, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        // Intentionally empty - do nothing
                    }
                });
            }
        });

        statusButton = (Button) findViewById(R.id.status_button);
		statusButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (controlNode != null)
                    controlNode.statusAll();
            }
        });

        stopButton = (Button) findViewById(R.id.stop_button);
		stopButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                confirm(activity, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        if (controlNode != null)
                            controlNode.stopAll();
                    }
                }, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        // Intentionally empty - do nothing
                    }
                });
            }
        });

        shutdownButton = (Button) findViewById(R.id.shutdown_button);
		shutdownButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                confirm(activity, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        if (controlNode != null)
                            controlNode.shutdownAll();
                    }
                }, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int buttonId) {
                        // Intentionally empty - do nothing
                    }
                });
            }
        });

        disableButtons();

		this.handler = new Handler() {
			@Override
			public void handleMessage(Message msg) {
                if (msg.obj instanceof std_msgs.String) {
                    std_msgs.String string = (std_msgs.String) msg.obj;

                    String data = string.getData();
                    TextView sayText = (TextView) findViewById(R.id.sayTextView);
                    sayText.setText(data);

                    // dlgAlert.setMessage("Message received for " + data);
                    // dlgAlert.setTitle("Notification");
                    // dlgAlert.setPositiveButton("OK", null);
                    // dlgAlert.setCancelable(true);
                    // dlgAlert.create().show();
                }
                else if (msg.obj instanceof UIUpdate) {
                    ((UIUpdate)msg.obj).update();
                }
				super.handleMessage(msg);
			}
		};
	}

    private void disableButtons() {
        stopListeningButton.setEnabled(false);
        resumeListeningButton.setEnabled(false);
        testSpeechButton.setEnabled(false);
        talkButton.setEnabled(false);
        resetButton.setEnabled(false);
        statusButton.setEnabled(false);
        stopButton.setEnabled(false);
        shutdownButton.setEnabled(false);
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
        SharedPreferences sharedPref = getPreferences(Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putString("ROS_MASTER_URI", ((EditText) findViewById(R.id.rosMasterUrlText)).getText().toString());
        editor.commit();
		if (!isRecognizingSpeech) {
            if (connectionState == ConnectionState.CONNECTED)
                doDisconnect((Button) findViewById(R.id.connectButton));
        }
	}

	@Override
	protected void onResume() {
		super.onResume();
        SharedPreferences sharedPref = getPreferences(Context.MODE_PRIVATE);
        String rosMasterURI = sharedPref.getString("ROS_MASTER_URI", "http://192.168.8.100:11311/");
        ((EditText) findViewById(R.id.rosMasterUrlText)).setText(rosMasterURI);

		if (!isRecognizingSpeech) {
//			executor = DefaultNodeMainExecutor.newDefault();
//			controlNode = newNode();
//			executor.execute(controlNode, nodeConfiguration);
		}
		isRecognizingSpeech = false;
	}

	private HumanoidControlNode newNode() {
		return new HumanoidControlNode("humanoid_control_node", "say");
	}

    private void connectDisconnectNode(View view) {
        Button connectButton = (Button) view;
        switch (connectionState) {
            case DISCONNECTED:
                doConnect(connectButton);
                break;
            case CONNECTING:
            case CONNECTED:
                doDisconnect(connectButton);
                break;
        }
    }

    private void doConnect(Button connectButton) {
        EditText rosMasterUrlText = (EditText) findViewById(R.id.rosMasterUrlText);
        String uri = rosMasterUrlText.getText().toString();
        try {
            String subnetFilter = uri.replaceFirst(".*http://", "").replaceFirst("(\\d+\\.\\d+\\.\\d+\\.).*", "$1");
            nodeConfiguration = NodeConfiguration.newPublic(Util.getIpAddress(subnetFilter), URI.create(uri));
            executor = DefaultNodeMainExecutor.newDefault();
            controlNode = newNode();
            executor.execute(controlNode, nodeConfiguration);
            connectionState = ConnectionState.CONNECTING;
            TextView connectionStatusView = (TextView) findViewById(R.id.connectionStatusView);
            connectionStatusView.setText("Connecting");
            connectionStatusView.setTextColor(0xFF00FF00);
            connectButton.setText("Disconnect");
            // TODO enable buttons
        }
        catch (Exception e) {
            dlgAlert.setTitle("Error");
            String message = e.getMessage();
            if (e.getCause() != null) {
                message = message + "\nCaused by: " + e.getCause().getMessage();
            }
            dlgAlert.setMessage(e.getMessage());
            dlgAlert.setPositiveButton("OK", null);
            dlgAlert.setCancelable(false);
            dlgAlert.create().show();
        }
    }

    private void doDisconnect(Button connectButton) {
        executor.shutdown();
        connectionState = ConnectionState.DISCONNECTED;
        nodeConfiguration = null;
        controlNode = null;
        TextView connectionStatusView = (TextView) findViewById(R.id.connectionStatusView);
        connectionStatusView.setText("Not connected.");
        connectionStatusView.setTextColor(0xffff0000);
        connectButton.setText("Connect");
        disableButtons();
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
                if (controlNode != null)
				    controlNode.publishRecognizedText(matches.get(0));
			}
		}
		super.onActivityResult(requestCode, resultCode, data);
	}
}
