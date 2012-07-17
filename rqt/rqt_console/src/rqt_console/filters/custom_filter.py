# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from QtCore import QObject, Signal

from ..message import Message

from severity_filter import SeverityFilter
from topic_filter import TopicFilter
from node_filter import NodeFilter
from message_filter import MessageFilter

class CustomFilter(QObject):
    """
    Contains filter logic for a single filter
    """
    filter_changed_signal = Signal()
    def __init__(self):
        super(CustomFilter, self).__init__()
        self._enabled = True
        
        self._message = MessageFilter()
        self._message.filter_changed_signal.connect(self.relay_emit_signal)
        self._severity = SeverityFilter()
        self._severity.filter_changed_signal.connect(self.relay_emit_signal)
        self._node = NodeFilter()
        self._node.filter_changed_signal.connect(self.relay_emit_signal)
        self._topic = TopicFilter()
        self._topic.filter_changed_signal.connect(self.relay_emit_signal)

    def set_enabled(self, checked):
        self._enabled = checked
        self._message.set_enabled(checked)
        self._severity.set_enabled(checked)
        self._node.set_enabled(checked)
        self._topic.set_enabled(checked)
        self.filter_changed_signal.emit()
    
    def relay_emit_signal(self):
        self.filter_changed_signal.emit()

    def is_enabled(self):
        return self._enabled

    def message_test(self, message):
        """
        Tests if the message matches the filter.
        
        :param message: the message to be tested against the filters, ''Message''
        :returns: True if the message matches, ''bool''
        """
        return self._message.message_test(message) and self._severity.message_test(message) and self._node.message_test(message) and self._topic.message_test(message)

