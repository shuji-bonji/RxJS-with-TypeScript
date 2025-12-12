---
description: O operador retry reinscreve e tenta novamente a fonte um número especificado de vezes quando ocorre um erro no Observable. Isso é útil para recuperar de falhas de comunicação temporárias, como falhas de rede, ou para processos que podem ter sucesso se tentados novamente após uma falha.
---

# retry - Tentar Novamente em Caso de Erro

O operador `retry` é um operador que **reinscreve o Observable fonte um número especificado de vezes** quando ocorre um erro.
É adequado para **processos que podem ter sucesso se tentados novamente após falha**, como falhas de rede temporárias.

## 🔰 Sintaxe Básica e Operação

### retry(count) - Forma Básica

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Erro temporário'))
  .pipe(
    retry(2), // Tentar novamente até 2 vezes
    catchError((error) => of(`Erro final: ${error.message}`))
  )
  .subscribe(console.log);
// Saída:
// Erro final: Erro temporário
```

Neste exemplo, até duas novas tentativas são feitas após a primeira falha, e uma mensagem é gerada em fallback se todas falharem.

### retry(config) - Formato de Objeto de Configuração (RxJS 7.4+)

No RxJS 7.4 e posterior, controle mais detalhado é possível passando um objeto de configuração.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Erro temporário'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Tentativa ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Tentar novamente até 2 vezes
      delay: 1000,        // Aguardar 1 segundo antes de tentar novamente (usa asyncScheduler internamente)
      resetOnSuccess: true // Redefinir contagem em caso de sucesso
    }),
    catchError((error) => of(`Erro final: ${error.message}`))
  )
  .subscribe(console.log);

// Saída:
// Tentativa 1
// Tentativa 2
// Tentativa 3
// Erro final: Erro temporário
```

> [!NOTE] Controle de Timing de Nova Tentativa
> Quando a opção `delay` é especificada, **asyncScheduler** é usado internamente. Para controle de timing de nova tentativa mais detalhado (backoff exponencial, etc.), consulte [Tipos de Scheduler e Uso - Controle de Nova Tentativa de Erro](/pt/guide/schedulers/types#error-retry-control).

[🌐 Documentação Oficial do RxJS - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Exemplo de Uso Típico

O exemplo a seguir é uma configuração que tenta novamente **processamento assíncrono com sucesso/falha aleatório** até 3 vezes.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Falha #${attempt}`));
      } else {
        return of(`Sucesso #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Falha final: ${err.message}`))
  )
  .subscribe(console.log);
// Saída:
// Sucesso #1
// Sucesso #5
// Sucesso #6
// Falha final: Falha #7
```

## 🧪 Exemplo de Código Prático (com UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Área de exibição de saída
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>Exemplo de retry (Simulação de Solicitação de API):</h3>';
document.body.appendChild(retryOutput);

// Exibição de status de solicitação
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// Solicitação de API que tem sucesso ou falha aleatoriamente
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Tentativa #${attemptCount} Enviando solicitação...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Tentativa #${attemptCount} Falhou: Erro de rede`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Erro de rede'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Tentativa #${attemptCount} Sucesso!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Dados recuperados com sucesso' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Todas as tentativas falharam: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Nova tentativa falhou' });
    })
  );
}

// Botão de início de solicitação
const startButton = document.createElement('button');
startButton.textContent = 'Iniciar Solicitação';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Resultado final: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Resultado final: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Resumo

- `retry(n)` tenta novamente até `n` vezes se o Observable falhar
- `retry` é **tentado novamente até completar com sucesso** (falha contínua resulta em erro)
- Útil para **APIs assíncronas e solicitações de rede** onde ocorrem falhas temporárias
- Comumente combinado com `catchError` para especificar **processamento de fallback**
- A partir do RxJS 7.4+, é possível especificar `delay`, `resetOnSuccess`, etc. em formato de objeto de configuração

## Páginas Relacionadas

- [retry e catchError](/pt/guide/error-handling/retry-catch) - Padrões para combinar retry e catchError, exemplos de uso prático
- [Depuração de Retry](/pt/guide/error-handling/retry-catch#retry-debugging) - Como rastrear contagem de tentativas (5 padrões de implementação)
- [Tipos de Scheduler e Uso](/pt/guide/schedulers/types#error-retry-control) - Controle de timing de nova tentativa detalhado, implementação de backoff exponencial
- [Técnicas de Depuração do RxJS](/pt/guide/debugging/#scenario-6-track-retry-attempt-count) - Cenários de depuração de nova tentativa
