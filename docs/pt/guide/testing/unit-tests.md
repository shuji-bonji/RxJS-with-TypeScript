---
description: Os testes unitários no RxJS usam técnicas síncronas, assíncronas e controladas por tempo para construir uma estratégia de teste robusta usando TestScheduler, marble testing, e mocks e stubs.
---

# Testes Unitários para RxJS

O código usando RxJS envolve muito processamento assíncrono e requer uma abordagem diferente dos métodos de teste tradicionais. Este guia descreve técnicas básicas e avançadas para testar efetivamente código usando RxJS.

## Testando Observable Síncrono

Vamos começar com o caso mais simples: testar um Observable que completa de forma síncrona.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Função sob teste
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Teste básico de Observable', () => {
  it('Dobra valores', () => {
    // Observable de teste
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Resultado esperado
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Execução e verificação
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Como testar um Observable assíncrono

Para Observable assíncrono, aproveite o suporte assíncrono do framework de teste.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Função assíncrona sob teste
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Testando Observable assíncrono', () => {
  it('Recebe valores assíncronos em ordem', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];

    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Teste assíncrono com transformação Promise

Outro método é converter um Observable para uma Promise usando `firstValueFrom()` ou `lastValueFrom()` e utilizar async/await do JS/TS moderno.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Função sob teste
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Testando com conversão Promise', () => {
  it('Aguarda processamento com delay antes da validação', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Converte Observable para promise
    const result = await lastValueFrom(result$);

    // Resultado esperado
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Utilizando TestScheduler

O RxJS fornece um scheduler especial chamado `TestScheduler` que pode ser usado para testar eficientemente operadores baseados em tempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Usando TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Testando debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );

      const expected = '----------(d|)';

      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> Notação Marble Test
> Ao usar `TestScheduler`, use diagramas marble para representar a passagem do tempo.

## Tornar o tempo manipulável

Ao testar código dependente de tempo (delay, debounceTime, etc.), use o `TestScheduler` para controlar o tempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Controle de tempo', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Avançar tempo rapidamente para teste', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Na realidade leva 3 segundos, mas executa imediatamente no ambiente de teste
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Testar tratamento de erros (versão TestScheduler)

Também é importante testar o comportamento do Observable quando um erro ocorre.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Teste de tratamento de erros', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Quando Observable notifica um erro', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('Quando catchError captura erro e substitui por um valor', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## Marble test

Para testar streams complexos, use um diagrama marble para representar intuitivamente as expectativas do teste.

### Hot Observable vs. Cold Observable

O TestScheduler permite a criação de dois tipos de Observables: hot e cold. É importante entender essa diferença ao testar.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Teste Hot vs Cold Observable', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable cria streams independentes para cada subscription', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (independente para cada subscriber)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Primeira subscription
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Segunda subscription (inicia do começo)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable compartilha streams entre subscribers', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (compartilhado entre subscribers)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Subscreve tarde (recebe apenas valores após o início da subscription)
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Subscreve desde o início (recebe todos os valores)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Testando Hot Observable usando Subject real', () => {
    // Versão não-TestScheduler
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Primeiro subscriber
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Emite valores
    subject.next(1);
    subject.next(2);

    // Segundo subscriber (entra no meio)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Emite mais valores
    subject.next(3);
    subject.complete();

    // Verificação
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Apenas valores após o início da subscription

    // Limpeza
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observable gera dados independentemente cada vez que você se inscreve, mas Hot Observable compartilha e distribui dados.

## Mocking e Stubbing

### Fazendo Mock de Serviços Dependentes

Ao testar serviços usando RxJS, é comum fazer mock de dependências externas.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Serviço sob teste
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Teste de serviço', () => {
  it('Filtra apenas usuários ativos', () => {
    // Mock do serviço API
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verificação
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Stubs

Stubs são objetos simples que imitam dados externos ou APIs dos quais o código sob teste depende.
Eles eliminam dependências de recursos externos e permitem que os testes sejam executados de forma independente.
Eles simplesmente retornam valores fixos e não têm lógica interna.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Serviço sob teste
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Teste de UserService', () => {
  it('Retorna apenas usuários ativos', () => {
    // 🔹 Criando stubs
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Serviço sob teste
    const userService = new UserService(stubApiService);

    // Verifica resultado
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Espiar subscriptions

Spy pode ser usado para verificar se as subscriptions estão sendo feitas corretamente.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Teste de subscription', () => {
  it('Inscrevendo com handlers apropriados', () => {
    const subject = new Subject();

    // Cria spies de handler
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Subscreve
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Emite valores
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verificação
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Melhores Práticas

|Melhores práticas|Explicação|
|---|---|
|Observe o princípio da responsabilidade única| Para escrever código testável, cada função ou classe deve ter uma única responsabilidade. Dessa forma, o teste é simplificado. |
|Faça mock de dependências externas|Dependências externas como requisições http e timers devem ter mock feito e serem testadas em um ambiente previsível. |
|Use técnicas apropriadas para código assíncrono| Escolha métodos apropriados para teste assíncrono, como TestScheduler, callbacks done() ou async/await. |
|Utilize marble testing| Para testar streams complexos, use diagramas marble para representar expectativas de teste de maneira intuitiva.|

## Resumo

Testar código RxJS tem aspectos que diferem do código JavaScript tradicional, como sua natureza síncrona/assíncrona e comportamento dependente de tempo. Ao escolher uma metodologia de teste apropriada, você pode desenvolver código reativo de alta qualidade com confiança. Em particular, mantenha os seguintes pontos em mente:

- Testes de subscription simples para Observable síncrono
- TestScheduler ou transformações Promise para processamento assíncrono
- Marble test para código dependente de tempo
- Faça mock de dependências externas para criar um ambiente de teste independente
- Projete código testável de acordo com o princípio da responsabilidade única

## 🔗 Seções Relacionadas

- **[Erros Comuns e Soluções](/pt/guide/anti-patterns/common-mistakes#15-lack-of-testing)** - Verifique anti-patterns relacionados a testes
- **[Utilizando TestScheduler](/pt/guide/testing/test-scheduler)** - Uso mais detalhado do TestScheduler
- **[Marble Testing](/pt/guide/testing/marble-testing)** - Técnicas avançadas de marble testing
