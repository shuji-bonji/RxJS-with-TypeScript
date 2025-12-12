---
description: O TestScheduler é uma ferramenta poderosa que permite testar operadores baseados em tempo do RxJS usando tempo virtual. Este guia explica a notação marble, como lidar com Cold e Hot Observable, e como testar com precisão processos dependentes de tempo como debounceTime e delay.
---

# Testando com TestScheduler

O `TestScheduler` do RxJS é uma ferramenta poderosa para testar com precisão operadores baseados em tempo. Este capítulo explica sistematicamente como testar utilizando o TestScheduler.

## O que é o TestScheduler?

Normalmente, os Observable funcionam de forma dependente do tempo. Por exemplo, `delay()` e `debounceTime()` são operadores que esperam por um determinado tempo.
Como é ineficiente aguardar de fato nos testes, o `TestScheduler` é um mecanismo para testar imediatamente usando tempo virtual.

> [!TIP]
> O TestScheduler usa "tempo virtual", portanto não há necessidade de esperar pelo tempo real.

## Configuração básica do TestScheduler

Esta é a configuração básica de teste usando TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Fundamentos do TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Teste simples', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Cria um Cold Observable onde o stream inicia independentemente a cada subscription
- `hot()`: Cria um Hot Observable onde o stream já está em andamento
- `expectObservable()`: verifica a saída do Observable com notação marble


## Cold Observable e Hot Observable

|Tipo|Características|Uso|
|:---|:---|:---|
|Cold Observable|Fluxo de dados desde o início a cada subscription|Requisição HTTP, etc.|
|Hot Observable|Fluxo de dados já iniciado e compartilhado com subscribers|Eventos do usuário, WebSockets, etc.|


## Fundamentos da notação marble

A notação marble é um método de representar a passagem do tempo em Observable como uma string.

|Símbolo|Significado|
|:---|:---|
|`-`|Tempo decorrido (um frame)|
|`a`, `b`, `c`|valor emitido|
|`|`|Completado|
|`#`|Erro|
|`() `|Múltiplos valores emitidos simultaneamente (múltiplos eventos)|

#### Exemplo

```
--a--b--c|    // a após 2 frames, então b, c, completa
```


## Exemplo de teste com tempo virtual

### Testando com debounceTime

Teste o operador debounceTime usando tempo virtual.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Testando com tempo virtual', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Testar operador debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ← Isto!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```


## Testar tratamento de erros

Teste o comportamento quando um erro ocorre.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Teste de tratamento de erros', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Capturar erro com catchError', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```


## Resumo

- O TestScheduler permite testar sem esperar pelo tempo real
- Entenda a diferença entre Cold/Hot Observable e use-os de forma diferente
- Visualize a passagem do tempo usando notação marble
- Teste até streams assíncronos complexos com precisão

> [!NEXT]
> A seguir, aprenderemos marble testing mais avançado (personalização de strings marble e combinações de múltiplos streams).
