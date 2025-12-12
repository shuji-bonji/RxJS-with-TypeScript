---
description: Esta apresentação cobre casos de uso práticos de operadores utilitários do RxJS (tap, startWith, finalize, delay, timeout, retry, etc.). Padrões práticos frequentemente usados no desenvolvimento de UI, como gerenciamento de estado de carregamento, validação de formulário reativo, controle de chamadas de API, tratamento de erros, assistência de depuração, etc., serão introduzidos com exemplos de código TypeScript. Você aprenderá técnicas de implementação para controlar e observar o comportamento do stream.
---

# Casos de Uso Práticos

## Gerenciando Estado de Carregamento

Este é um exemplo de uso de `tap`, `finalize`, etc. para gerenciar o estado de carregamento.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// Elementos da UI
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>Chamada de API e gerenciamento de estado de carregamento:</h3>';
document.body.appendChild(loadingExample);

// Indicador de carregamento
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Carregando...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Área de exibição de dados
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Botão de sucesso
const successButton = document.createElement('button');
successButton.textContent = 'Solicitação bem-sucedida';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Botão de falha
const failButton = document.createElement('button');
failButton.textContent = 'Solicitação com falha';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simular solicitação de API bem-sucedida
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Dados de exemplo',
    description: 'Estes são dados recuperados da API.'
  }).pipe(
    // Mostrar carregamento no início da solicitação
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simular latência da API
    delay(1500),
    // Sempre ocultar carregamento na conclusão da solicitação
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simular solicitação de API com falha
function simulateFailRequest() {
  return throwError(() => new Error('Solicitação de API falhou')).pipe(
    // Mostrar carregamento no início da solicitação
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simular latência da API
    delay(1500),
    // Tratamento de erro
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Erro: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Sempre ocultar carregamento na conclusão da solicitação
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Clique no botão de sucesso
successButton.addEventListener('click', () => {
  // Desabilitar botões
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Exibir dados
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Erro:', err);
    },
    complete: () => {
      // Reabilitar botões
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Clique no botão de falha
failButton.addEventListener('click', () => {
  // Desabilitar botões
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // Não terá sucesso, mas só por precaução
    },
    error: () => {
      // Erro já tratado por catchError
      console.log('Tratamento de erro concluído');
    },
    complete: () => {
      // Reabilitar botões
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Validação e Envio de Formulário

A seguir está um exemplo de implementação de validação e envio de formulário usando `startWith`, `tap`, `finalize`, etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// UI do formulário
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Exemplo de formulário reativo:</h3>';
document.body.appendChild(formExample);

// Criar elementos do formulário
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Campo de entrada de nome
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nome: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.padding = '8px';
nameInput.style.width = '100%';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

const nameError = document.createElement('div');
nameError.style.color = 'red';
nameError.style.fontSize = '12px';
nameError.style.marginTop = '-10px';
nameError.style.marginBottom = '15px';
form.appendChild(nameError);

// Campo de entrada de email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Endereço de email: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.padding = '8px';
emailInput.style.width = '100%';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

const emailError = document.createElement('div');
emailError.style.color = 'red';
emailError.style.fontSize = '12px';
emailError.style.marginTop = '-10px';
emailError.style.marginBottom = '15px';
form.appendChild(emailError);

// Botão de envio
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Enviar';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Inicialmente desabilitado
form.appendChild(submitButton);

// Área de exibição de resultado
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Validação de entrada de nome
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Nome é obrigatório' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Nome deve ter pelo menos 2 caracteres' };
    }
    return { value, valid: true, error: null };
  })
);

// Validação de entrada de email
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Endereço de email é obrigatório' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Por favor, insira um endereço de email válido' };
    }
    return { value, valid: true, error: null };
  })
);

// Monitorar estado de validação do formulário completo
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // Todo o formulário é válido
    const isValid = nameState.valid && emailState.valid;

    // Exibir erros de validação
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Habilitar/desabilitar botão de envio
  submitButton.disabled = !isValid;
});

// Processamento de envio de formulário
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Prevenir envio padrão do formulário
    event.preventDefault();

    // Definir para estado de envio
    submitButton.disabled = true;
    submitButton.textContent = 'Enviando...';

    // Redefinir área de exibição de resultado
    formResult.style.display = 'none';
  }),
  // Obter dados do formulário
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simular solicitação de API
  delay(1500),
  // Sempre retornar ao estado de envio concluído
  finalize(() => {
    submitButton.textContent = 'Enviar';
    submitButton.disabled = false;
  }),
  // Tratamento de erro
  catchError(error => {
    formResult.textContent = `Erro: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Continuar stream
  })
).subscribe(data => {
  if (data) {
    // Envio bem-sucedido
    formResult.innerHTML = `
      <div style="font-weight: bold;">Envio bem-sucedido!</div>
      <div>Nome: ${data.name}</div>
      <div>Email: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Redefinir formulário
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Como Escolher um Operador Utilitário

| Propósito | Operador | Situação de Uso |
|------|--------------|---------|
| Execução de efeito colateral | `tap` | Depuração, saída de log, atualização de UI, etc. |
| Atraso na saída de valores | `delay` | Animação, ajuste de tempo, etc. |
| Configurações de timeout | `timeout` | Timeout para solicitações de API, processamento assíncrono |
| Processamento na conclusão | `finalize` | Limpeza de recursos, liberar estado de carregamento |
| Definir valor inicial | `startWith` | Inicializar estado, exibir placeholders |
| Converter para um array | `toArray` | Processamento em lote, todos os resultados são processados juntos |
| Tentar novamente em caso de erro | `retry` | Solicitações de rede, recuperação de erros temporários |
| Repetir um stream | `repeat` | Polling, processamento periódico |

## Resumo

Operadores utilitários são ferramentas importantes que tornam a programação em RxJS mais eficiente e robusta. A combinação adequada desses operadores oferece os seguintes benefícios:

1. **Facilidade de Depuração**: Usando `tap`, você pode facilmente verificar o estado intermediário do stream.
2. **Tolerância a Erros**: A combinação de `retry`, `timeout` e `catchError` fornece tratamento de erros robusto.
3. **Gerenciamento de Recursos**: `finalize` pode ser usado para garantir a limpeza adequada de recursos.
4. **Melhor capacidade de resposta da UI**: `startWith`, `delay`, etc. podem ser usados para melhorar a experiência do usuário.
5. **Melhorar a legibilidade do código**: O uso de operadores utilitários pode separar claramente efeitos colaterais da conversão pura de dados.

Esses operadores demonstram seu verdadeiro valor quando usados em combinação com outros operadores, em vez de sozinhos. No desenvolvimento real de aplicações, é comum combinar múltiplos operadores para gerenciar fluxos de processamento assíncrono complexos.
