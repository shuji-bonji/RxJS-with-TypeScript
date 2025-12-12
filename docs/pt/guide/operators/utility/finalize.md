---
description: finalize é um operador utilitário do RxJS que define um processo a ser executado sempre que um Observable é completado, gera erro ou é cancelado. É ideal para situações que requerem limpeza no final do stream, como liberação de recursos, término de exibição de carregamento e operações de limpeza. Garante que as operações sejam executadas de forma tão confiável quanto try-finally e ajuda a prevenir vazamentos de memória.
---

# finalize - Processamento na Conclusão

O operador `finalize` define um processo que é chamado sempre que **Observable é completado, gera erro ou é cancelado**.
Isso é ideal para processos que "devem ser executados", como limpeza e liberação de carregamento da UI.

## 🔰 Sintaxe Básica e Operação

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Completado')
  .pipe(finalize(() => console.log('Stream foi encerrado')))
  .subscribe(console.log);
// Saída:
// Completado
// Stream foi encerrado
```

Neste exemplo, o processo em `finalize` é executado após emitir um valor em `of()`.
**Ele é chamado de forma confiável tanto para `complete` quanto para `error`**.

[🌐 Documentação Oficial do RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Exemplo de Uso Típico

A seguir está um exemplo de alternância de exibição de carregamento antes e depois do streaming.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Dados')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Carregamento iniciado');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Carregamento finalizado');
    })
  )
  .subscribe((value) => console.log('Recuperado:', value));
// Saída:
// Carregamento iniciado
// Recuperado: Dados
// Carregamento finalizado
```

## 🧪 Exemplo de Código Prático (com UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Área de exibição de saída
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>Exemplo de finalize:</h3>';
document.body.appendChild(finalizeOutput);

// Indicador de carregamento
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Carregando dados...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Exibição de progresso
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Elemento de mensagem de conclusão
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Simulação de recuperação de dados
interval(500)
  .pipe(
    take(5), // Recuperar 5 valores
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Processando item ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Processamento concluído!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Todos os dados foram carregados com sucesso.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Resumo

- `finalize` é **sempre executado** independentemente de conclusão, erro ou término manual
- Ideal para processos de limpeza e término de carregamento
- Pode ser combinado com outros operadores (`tap`, `delay`, etc.) para **limpeza assíncrona segura**
