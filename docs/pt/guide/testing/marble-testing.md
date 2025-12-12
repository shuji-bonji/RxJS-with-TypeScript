---
description: O marble testing é uma técnica que permite testar streams assíncronos do RxJS com uma representação visual usando strings. Este guia explica cuidadosamente a diferença entre Cold e Hot Observable, as regras da notação marble e como usar o TestScheduler desde exemplos básicos até práticos.
---

# Introdução ao Marble Testing

O RxJS oferece uma técnica chamada "marble testing" que permite testar o comportamento de streams assíncronos **com uma representação visual**.

Nesta seção, aprenderemos os fundamentos do marble testing através de exemplos simples.

## O que é a Notação Marble?

A notação marble é uma forma de representar **a passagem do tempo e a ocorrência de eventos** com strings.

### Regras básicas

Estes símbolos são usados para representar a passagem do tempo e a ocorrência de eventos.

| Símbolo | Significado |
|:----|:----|
| `-` | O tempo passa (avança 1 frame) |
| `a`, `b`, `c` | Valores emitidos (caracteres arbitrários) |
| `|` | Completo |
| `#` | Erro |

Por exemplo:

```text
--a-b--c-|
```
Isto significa:
- Aguarde 2 frames e `a` é emitido
- `b` após 1 frame
- `c` 2 frames depois
- Completa após mais 1 frame

## Diferença entre Cold e Hot

### Cold Observable

Cold Observable é "reproduzido desde o início a cada subscription".

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

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Um Hot Observable é um stream que "já está em andamento".
Se você se inscrever no meio de um stream, receberá apenas valores daquele ponto em diante.

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
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## Um exemplo simples de marble test

Por exemplo, para testar o operador `debounceTime`:

```ts
import { debounceTime } from 'rxjs';
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
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Aqui verificamos que apenas o último `c` emitido é produzido.

## Observações

- Um caractere na notação marble representa **um frame (10ms)** por padrão (configurável dependendo do ambiente)
- Operadores dependentes de tempo como `debounceTime`, `delay`, `interval`, etc. **funcionam bem com marble tests**
- Use `expectObservable` para validar a saída do stream
- `expectSubscriptions` é um recurso avançado que valida o timing das subscriptions, mas não é abordado aqui

## Resumo

O marble testing é uma técnica muito poderosa que torna o teste de código RxJS **visível e intuitivo**.

- **Esteja ciente da diferença entre Cold e Hot**
- **Representar a passagem do tempo e eventos como strings**
- **Streams assíncronos complexos também podem ser testados claramente**

Vamos começar com um marble test simples para praticar!
