---
description: "O combineLatestWith é um operador de combinação RxJS que combina o Observable original com os valores mais recentes de outros Observables. É ideal para situações em que você deseja monitorar constantemente os valores mais recentes de vários fluxos dependentes, como validação em tempo real de entradas de formulários, sincronização de vários estados, atualizações em tempo real de resultados de cálculos etc. A versão do operador pipe é conveniente para uso em pipelines."
---

# combineLatestWith - combina os valores mais recentes

O operador `combineLatestWith` combina todos os **valores mais recentes** do Observable original e de qualquer outro Observable especificado.
Sempre que um novo valor é emitido de um dos Observable, é emitido um resultado que combina todos os valores mais recentes.
Essa é a versão do Pipeable Operator do `combineLatest` da Creation Function.

## Sintaxe básica e uso

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Exemplos de saída:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Depois que cada Observable tiver emitido **pelo menos um valor**, o valor combinado é emitido.
- Sempre que um novo valor chega em um dos lados, o último par é reemitido.

[🌐 Documentação oficial do RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Padrão de utilização típico.

- Validação em tempo real de entradas de formulário**: monitoramento constante do estado mais recente de vários campos
- Sincronização de vários estados dependentes**: combinação de valores de configuração e entrada do usuário
- Atualização em tempo real dos resultados de cálculo**: cálculo imediato dos resultados de vários valores de entrada

## Exemplos práticos de código (com UI)

Exemplo de cálculo do valor total em tempo real a partir de entradas de preço e quantidade.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Criação de área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Exemplos práticos de:</h3>';
document.body.appendChild(output);

// Criação de campos de entrada
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Preço unitário';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Quantidade';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Área de exibição de resultados
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// de cada inputObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Calculado pela combinação dos valores mais recentes
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Valor total: ¥${total.toLocaleString()}</strong>`;
  });
```

- Quando você insere um dos campos, o total é **imediatamente recalculado** a partir dos dois valores mais recentes.
- O método `startWith()` é usado para obter o resultado combinado desde o início.

## 🔄 Diferenças com a Creation Function `combineLatest`.

### Diferenças básicas.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Exemplos de saída:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Exemplos específicos de uso

**Se você quiser apenas combinações simples, a Creation Function é o caminho a seguir**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Simples e fácil de ler
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Idade)`);
});
// Saída: Yamada Taro (30Idade)
```

**Se você quiser adicionar um processo de conversão ao fluxo principal, o Pipeable Operator é recomendado**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Busca por...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Todos os livros</option><option>Livros</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Principal: Pesquisar palavras-chave
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Após a entrada300msAguardar
  startWith('')
);

// Subfluxo: Seleção de categoria
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Todos os livros')
);

// ✅ Pipeable OperatorEdição - Concluído em um pipeline
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Convertido em letras minúsculas
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Busca por: "${result.term}" Categoria: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionEdição - Tornou-se redundante
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Busca por: "${result.term}" Categoria: ${result.category} [${result.timestamp}]`;
});
```

**Ao combinar vários valores de configuração**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Criação de controle deslizante
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Principal: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorEdição - RedCombinar outras cores como principais
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Resumo.

- combineLatest: ideal se você quiser simplesmente combinar vários fluxos
- combineLatestWith: ideal se você quiser combinar os valores mais recentes de outros fluxos enquanto transforma ou processa o fluxo principal

## ⚠️ Notas.

### Não é emitido até que os valores iniciais estejam disponíveis.

Nenhum resultado é emitido até que todos os Observable tenham emitido pelo menos um valor.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Nenhum valor emitidoObservable
).subscribe(console.log);
// Nenhuma saída (porque)NEVERNenhuma saída (porque)
```

Isso pode ser resolvido fornecendo um valor inicial com `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Saída: [0, null] → [1, null] → [2, null]
```

### Cuidado com as reedições frequentes.

Se algum fluxo emitir valores com frequência, o resultado também será reemitido com frequência.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msFluxo emitido a cada
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$cada vez que um valor é emitido,fast$é emitido, ele é combinado com o valor mais recente de
// → O desempenho precisa de atenção
```

### Tratamento de erros.

Se ocorrer um erro em qualquer Observable, todo o processo terminará com um erro.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Ocorrem erros')).pipe(
      catchError((err: unknown) => of('Recuperação'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Saída: [0, 'Recuperação'] → [1, 'Recuperação']
```

## 📚 Operadores relacionados.

- **[combineLatest](/pt/guide/creation-functions/combination/combineLatest)** - versão da Creation Function.
- **[withLatestFrom](/pt/guide/operators/combination/withLatestFrom)** - acionado somente pelo mainstream
- **[zipWith](/pt/guide/operators/combination/zipWith)** - Emparelha valores correspondentes
