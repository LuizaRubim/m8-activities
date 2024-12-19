import streamlit as st
from dotenv import load_dotenv
import os
from llama_index.core import SimpleDirectoryReader, VectorStoreIndex, Settings, StorageContext, load_index_from_storage
from llama_index.llms.gemini import Gemini
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.core.node_parser import SentenceSplitter
import time


load_dotenv()
api_key = os.environ['GOOGLE_API_KEY']
persist_dir = os.environ['PERSIST_DIR']
text_splitter = SentenceSplitter(chunk_size=1024, chunk_overlap=20)

llm = Gemini(
    model="models/gemini-1.5-flash",
    api_key=api_key,
)
embed_model = HuggingFaceEmbedding(model_name="BAAI/bge-small-en-v1.5")
Settings.llm = llm
Settings.text_splitter = text_splitter
Settings.embed_model = embed_model

def load_documents(directory):
    loader = SimpleDirectoryReader(directory)
    documents = loader.load_data()
    return documents

def build_index():
    if not os.path.exists(persist_dir):
        documents = load_documents("./documents")
        index = VectorStoreIndex.from_documents(documents)
        index.storage_context.persist(persist_dir= persist_dir)
        return index
    else:
        storage_context = StorageContext.from_defaults(persist_dir=persist_dir)
        index = load_index_from_storage(storage_context)
        return index

def query_index(starting_engine, query):    
    response = starting_engine.query(query)
    return response.response

def response_generator(response):
    for word in response.split():
        yield word + " "
        time.sleep(0.05)

def main():
    st.set_page_config(page_title="Industrial Safety Chatbot", layout="centered")
    st.title("Industrial Safety Standards Chatbot 🦾")

    if "index" not in st.session_state:
        st.session_state.index = build_index()

    if "messages" not in st.session_state:
        st.session_state.messages = []

    for message in st.session_state.messages:
        with st.chat_message(message["role"]):
            st.markdown(message["content"])

    system_prompt = (
        "You are an assistant specialized in industrial safety standards. "
        "Respond succinctly and clearly using the information provided. "
        "If a question is irrelevant or out of scope, politely respond that you cannot assist."
    )

    starting_engine = st.session_state.index.as_query_engine(system_prompt=system_prompt, similarity_top_k=5)

    if prompt := st.chat_input("How can I help you today?"):
        st.session_state.messages.append({"role": "user", "content": prompt})
        with st.chat_message("user"):
            st.markdown(prompt)

        with st.spinner("Loading..."):
            context_response = query_index(starting_engine, prompt)
        with st.chat_message("assistant"):
            response = st.write_stream(response_generator(context_response))
            st.session_state.messages.append({"role": "assistant", "content": response})


if __name__ == "__main__":
    main()
