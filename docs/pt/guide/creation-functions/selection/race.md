---
description: A Creation Function race alcança um processo especial de concatenação que adota apenas o primeiro stream que emite um valor entre vários Observables e ignora os outros posteriormente.
titleTemplate: ':title | RxJS'
---

# race - adotar o stream que emitiu o valor primeiro

`race` é uma Creation Function de concatenação especial que aproveita **apenas o primeiro Observable que emite um valor** entre vários Observables,
e ignora os outros Observables.


## Sintaxe básica e uso

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lento (5 segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (2 segundos)'));

race(slow$, fast$).subscribe(console.log);
// Saída: Rápido (2 segundos)
```

- Apenas o Observable que emitiu o valor primeiro é o vencedor e continua com os streams subsequentes.
- Outros Observables são ignorados.

[🌐 Documentação Oficial RxJS - `race`](https://rxjs.dev/api/index/function/race)


## Padrões típicos de utilização

- **Processar a primeira de várias ações do usuário (cliques, teclas, rolagem)**
- **Adotar a primeira de vários gatilhos, como envio manual e salvamento automático**
- **Exibir os dados concluídos primeiro entre vários processos de aquisição de dados**

## Exemplos de código prático (com UI)

Simula uma corrida para adotar apenas o primeiro emitido de três streams disparando em momentos diferentes.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de race:</h3>';
document.body.appendChild(output);

// Observables com temporização diferente
const slow$ = timer(5000).pipe(map(() => 'Lento (após 5 segundos)'));
const medium$ = timer(3000).pipe(map(() => 'Médio (após 3 segundos)'));
const fast$ = timer(2000).pipe(map(() => 'Rápido (após 2 segundos)'));

const startTime = Date.now();

// Mensagem de início da corrida
const waiting = document.createElement('div');
waiting.textContent = 'Corrida iniciada... Aguardando o primeiro stream ser emitido.';
output.appendChild(waiting);

// Executar corrida
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Vencedor:</strong> ${winner} (Tempo decorrido: ${elapsed} segundos)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Apenas o primeiro Observable que emitiu um valor é selecionado.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Após 2 segundos, o primeiro `fast$` é emitido e, posteriormente, apenas `fast$` é exibido.
- Outras emissões de `medium$` e `slow$` serão ignoradas.


## Operadores Relacionados

- **[raceWith](/pt/guide/operators/combination/raceWith)** - Versão Pipeable Operator (usada em pipeline)
- **[timeout](/pt/guide/operators/utility/timeout)** - Operador somente de timeout
- **[merge](/pt/guide/creation-functions/combination/merge)** - Creation Function que mescla todos os streams
