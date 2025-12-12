---
description: timeout é um utility operator do RxJS que lança um erro se nenhum valor for emitido do Observable dentro de um tempo especificado. Ideal para processamento reativo com restrição de tempo, como controle de timeout de solicitação de API, espera de respostas de ação do usuário ou detecção de atraso de stream. Pode ser combinado com catchError para implementar comportamento de fallback, e a inferência de tipo do TypeScript permite processamento de timeout com segurança de tipo.
---

# timeout - Configuração de Timeout

O operador `timeout` é um operador que **lança um erro se nenhum valor for emitido pelo Observable dentro de um tempo especificado**.
É frequentemente usado para processamento reativo, como esperar uma resposta a uma solicitação de API ou operação do usuário.


## 🔰 Sintaxe Básica e Operação

Se o timeout não for excedido, a operação continua normalmente; se exceder um determinado período, ocorre um erro.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('resposta')
  .pipe(
    delay(500), // 👈 Se definido como 1500, gera `Erro de timeout: fallback`
    timeout(1000),
    catchError((err) => of('Erro de timeout: fallback', err))
  )
  .subscribe(console.log);
// Saída:
// resposta
```

Neste exemplo, `'resposta'` é normalmente exibida já que o valor é emitido após 500ms devido a `delay(500)` e a condição de `timeout(1000)` é satisfeita.

Se `delay(1200)` for especificado, um `erro de timeout` é gerado da seguinte forma:
```sh
Erro de timeout: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 Documentação Oficial do RxJS - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Exemplo de Uso Típico

O exemplo a seguir mostra tanto um **padrão que causa timeout se o stream atrasar e não emitir um valor** quanto um **padrão que emite normalmente**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout ocorreu'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout disparado'))
  )
  .subscribe(console.log);
// Saída:
// 0
// 1
// fallback: timeout disparado
// 2
```


## 🧪 Exemplo de Código Prático (com UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Área de exibição de saída
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>Exemplo de timeout:</h3>';
document.body.appendChild(timeoutOutput);

// Caso de sucesso de timeout
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Stream Normal (Sem Timeout):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Erro: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Valor de fallback após erro');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Caso de erro de timeout
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Stream Lento (Timeout Ocorre):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Erro: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Valor de fallback após timeout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Resumo

- `timeout` é um operador de controle que **lança um erro se não houver emissão dentro de um determinado tempo**
- Eficaz para processamento de timeout enquanto aguarda APIs de rede ou operações de UI
- Pode ser combinado com `catchError` para especificar **comportamento de fallback**
