# LLM Agent

Interfaces with cloud LLMs (OpenAI/Gemini) to convert natural language commands into structured mission plans.

## Overview

Takes natural language input and generates JSON mission plans that the drone can execute.

## Configuration

Set API keys via environment variables:
```bash
export OPENAI_API_KEY="your-key"
# or
export GEMINI_API_KEY="your-key"
```

## Topics

**Subscribers:**
- `/natural_language_command` (std_msgs/String) - Commands to process

**Publishers:**
- `/mission_plan` (std_msgs/String) - JSON mission plans

## Parameters

- `llm_provider` - "openai" or "gemini" (default: "openai")
- `api_key` - Override environment variable
