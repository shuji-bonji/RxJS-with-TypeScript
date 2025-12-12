---
description: Esta página fornece explicações práticas de técnicas de depuração RxJS sob as perspectivas de estratégias básicas, cenários comuns de depuração, ferramentas de depuração e depuração de desempenho.
---

# Técnicas de Depuração RxJS

Devido à natureza dos streams assíncronos, a depuração do RxJS requer uma abordagem diferente das técnicas tradicionais de depuração síncrona.

Esta página fornece estratégias básicas para depurar aplicações RxJS e navegação para técnicas de depuração detalhadas.

## Visão Geral das Técnicas de Depuração

A depuração do RxJS pode ser categorizada nas seguintes quatro abordagens:

| Abordagem | Conteúdo | Página de Detalhes |
|----------|------|-----------|
| **Estratégia Básica** | operador tap, ferramentas do desenvolvedor, RxJS DevTools | Explicado nesta página |
| **Cenários Comuns** | Seis problemas típicos: nenhum valor flui, vazamentos de memória, erros perdidos, etc. | [→ Detalhes](/pt/guide/debugging/common-scenarios) |
| **Ferramentas Personalizadas** | Streams nomeados, operadores de debug, medição de desempenho | [→ Detalhes](/pt/guide/debugging/custom-tools) |
| **Desempenho** | Monitoramento de subscription, detecção de reavaliação, verificação de uso de memória, melhores práticas | [→ Detalhes](/pt/guide/debugging/performance) |

## Estratégias básicas de depuração

### 1. Saída de log com operador `tap`

O operador `tap` é a técnica de depuração mais básica, permitindo observar valores de stream sem efeitos colaterais.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Valor original:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Após map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Após filter:', value))
  )
  .subscribe(value => console.log('✅ Valor final:', value));

// Saída:
// 🔵 Valor original: 0
// 🟢 Após map: 0
// 🔵 Valor original: 1
// 🟢 Após map: 2
// 🔵 Valor original: 2
// 🟢 Após map: 4
// 🔵 Valor original: 3
// 🟢 Após map: 6
// 🟡 Após filter: 6
// ✅ Valor final: 6
```

#### Pontos-Chave
- Insira um `tap` em cada etapa do pipeline para rastrear o fluxo de dados
- Use pictogramas e labels para melhorar a visibilidade do log
- Logs de debug podem ser inseridos com segurança porque `tap` não altera valores

### 2. Saída de informações detalhadas de log

Para obter informações de depuração mais detalhadas, use o objeto Observer.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Stream normal
of(1, 2, 3)
  .pipe(debug('Normal'))
  .subscribe();

// Saída:
// [Normal] next: 1
// [Normal] next: 2
// [Normal] next: 3
// [Normal] complete

// Stream com erro
concat(
  of(1, 2),
  throwError(() => new Error('Erro ocorreu'))
)
  .pipe(debug('Error'))
  .subscribe({
    error: () => {} // Tratamento de erro
  });

// Saída:
// [Error] next: 1
// [Error] next: 2
// [Error] error: Error: Erro ocorreu
```

### 3. Verificar com ferramentas do desenvolvedor

Esta é uma técnica de depuração que utiliza as ferramentas do desenvolvedor do navegador.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Função auxiliar para depuração
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Valor:', value);
      console.log('Tipo:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Depurando eventos de clique de botão
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Evento de Clique'),
      debounceTime(300),
      tapDebugger('Após Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Enviar:', data));
}
```

#### Utilizando Ferramentas do Desenvolvedor
- Agrupe logs com `console.group()`
- Exiba stack traces com `console.trace()`
- Exiba arrays e objetos em formato fácil de ler com `console.table()`
- Coloque breakpoints em `tap`

### 4. Utilizando RxJS DevTools

RxJS DevTools é uma ferramenta de depuração fornecida como extensão de navegador.

#### Instalação
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Principais recursos
- Visualização do status de subscription de Observable
- Exibição de timeline de valores de stream
- Detecção de vazamento de memória
- Análise de desempenho

#### Exemplo de Uso

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Habilitar depuração apenas no ambiente de desenvolvimento
// Diferentes build tools usam diferentes verificações de variável de ambiente
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Configuração manual: use variáveis globais
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Tornar observável no DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Técnicas de depuração detalhadas

Uma vez que você entende a estratégia básica, aprenda técnicas de depuração específicas nas páginas detalhadas abaixo.

### Cenários Comuns de Depuração

Seis problemas típicos encontrados no desenvolvimento do mundo real e como resolvê-los:

- Cenário 1: Nenhum valor flui
- Cenário 2: Valor diferente do esperado é emitido
- Cenário 3: Subscription nunca completa (stream infinito)
- Cenário 4: Vazamento de memória (esqueceu de fazer unsubscribe)
- Cenário 5: Erro ocorre e não é notado
- Cenário 6: Desejo rastrear tentativas de retry

[→ Ver Cenários Comuns de Depuração](/pt/guide/debugging/common-scenarios)

### Ferramentas de Debug Personalizadas

Como criar suas próprias ferramentas de depuração para atender aos requisitos do seu projeto:

- Depurando Streams Nomeados (tagStream)
- Criando operadores de debug personalizados
- Operador para medição de desempenho (measure)

[→ Ver Ferramentas de Debug Personalizadas](/pt/guide/debugging/custom-tools)

### Depuração de Desempenho

Otimização de aplicação e melhores práticas:

- Verificar e rastrear subscriptions
- Detectar reavaliações desnecessárias (shareReplay)
- Monitorar uso de memória
- Criando um Ambiente de Depuração
- Depuração type-safe
- Definir error boundaries

[→ Ver Depuração de Desempenho](/pt/guide/debugging/performance)

## Resumo

A depuração do RxJS pode ser feita de forma eficiente seguindo estes pontos.

### Estratégia Básica
- ✅ Observe cada estágio do stream com o operador `tap`
- ✅ Utilize ferramentas do desenvolvedor para saída de log detalhada
- ✅ Visualize o stream com RxJS DevTools

### Cenários Comuns
- ✅ Valores não fluem → Esqueceu subscription, verifique condições de filtragem
- ✅ Valor diferente do esperado → Ordem do operador, note compartilhamento de referência
- ✅ Subscription não completada → use `take` ou `takeUntil` para streams infinitos
- ✅ Vazamentos de memória → auto unsubscribe com padrão `takeUntil`
- ✅ Erros perdidos → implemente tratamento de erro adequado

### Ferramentas de Depuração
- ✅ Depuração flexível com operadores de debug personalizados
- ✅ Rastreie múltiplos streams com streams nomeados
- ✅ Identifique gargalos com medição de desempenho

### Desempenho
- ✅ Previna vazamentos de memória monitorando subscriptions
- ✅ Evite recálculos desnecessários com `shareReplay`
- ✅ Verifique uso de memória periodicamente

Combinadas, essas técnicas permitem a depuração eficiente de aplicações RxJS.

## Páginas Relacionadas

- [Tratamento de Erros](/pt/guide/error-handling/strategies) - Estratégias de tratamento de erros
- [Técnicas de Teste](/pt/guide/testing/unit-tests) - Como testar RxJS
- [Anti-padrões RxJS](/pt/guide/anti-patterns/) - Erros comuns e soluções
- [Pipeline](/pt/guide/operators/pipeline) - Encadeamento de operadores
